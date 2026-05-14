# Algoritmo do line follower

Documento focado **apenas nos scripts** do pacote — explica o pipeline
visual de ponta a ponta, da chegada do frame da câmera até a publicação
do comando de velocidade que faz a roda girar.

Os arquivos cobertos:

- [`scripts/line2.py`](scripts/line2.py) — o nó de controle (executável `line_follower`); contém todo o pipeline de visão + a malha de controle
- [`scripts/robot_controller.py`](scripts/robot_controller.py) — classe `bot_control`, encapsula a publicação de `TwistStamped` em `/cmd_vel`
- [`scripts/view.py`](scripts/view.py) — nó visualizador (executável `viewer`), opcional, mostra o que o robô vê

---

## Visão geral em uma frase

A cada frame de 320×240 que chega da câmera, o nó **corta a faixa
inferior**, **isola pixels escuros em HSV**, encontra o **centróide do
maior contorno**, calcula um **erro horizontal** entre esse centróide
e o centro da imagem, e usa esse erro para gerar um **comando
proporcional** (angular) com **velocidade adaptativa** (linear maior
na reta, menor na curva).

```
imagem da câmera ──► ROI ──► HSV ──► máscara ──► contornos ──► centróide
                                                                  │
                                                                  ▼
        TwistStamped ◄── bot_control.move(linear, angular) ◄── controle P
              │
              ▼
         /cmd_vel ──► bridge ──► gz diff_drive ──► rodas
```

---

## `line2.py` — pipeline completo

### Inicialização ([line2.py:25-37](scripts/line2.py#L25-L37))

```python
class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.bridge = cv_bridge.CvBridge()
        self.sub = self.create_subscription(
            Image, CAMERA_TOPIC, self.callback, 10)
        self.debug_pub = self.create_publisher(Image, DEBUG_TOPIC, 10)
        self.bc = bot_control(self)
```

O nó cria três coisas:

1. **`cv_bridge.CvBridge()`** — converte mensagens ROS `sensor_msgs/Image`
   em arrays NumPy do OpenCV (e vice-versa).
2. **Subscription** em `/camera/image_raw` (queue 10) — chama `self.callback` a cada frame.
3. **Publisher** em `/line_follower/debug_image` — frame anotado pra visualização.
4. **`bot_control(self)`** — passa o próprio nó pro helper; `bot_control` usará
   `self.create_publisher` do nó pra publicar `/cmd_vel`.

A partir daqui, toda a lógica vive dentro de `callback(data)`, que é chamado
~30× por segundo pelo executor do ROS.

---

### Passo 1 — Decodificar a mensagem ROS para um array OpenCV

```python
frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
```

- `data` é um `sensor_msgs/msg/Image` cru do bridge gz→ROS
- `cv_bridge` desserializa para `np.ndarray` de shape `(240, 320, 3)` no
  formato BGR (a ordem de canais que o OpenCV usa por convenção)
- A escolha `"bgr8"` força a conversão: a câmera publica em RGB
  (`R8G8B8` no SDF), o bridge entrega `rgb8`, e a string `"bgr8"`
  pede pro `cv_bridge` trocar para BGR antes de devolver

---

### Passo 2 — Recortar a ROI (Region of Interest)

```python
def _crop_bottom(self, frame):
    h = frame.shape[0]
    return frame[int(h * 0.6):h, :]

roi = self._crop_bottom(frame)
```

- Pega só os **40% inferiores** da imagem → shape `(96, 320, 3)`
- **Por quê:** a faixa inferior corresponde ao chão **logo à frente do
  robô**. A parte superior mostra o horizonte e linhas distantes, que
  distorcem o centróide (curvas longe puxam o centróide pra um lado
  enquanto a frente está reta) e atrasam a reação
- **Tradeoff:** mais ROI (ex.: 30%) ⇒ enxerga mais à frente, antecipa
  curva, mas reage mais ao "futuro"; menos ROI ⇒ reação local e
  estável, mas pode chegar "tarde" em curvas fechadas

---

### Passo 3 — Converter para HSV

```python
hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
```

- Converte de BGR (luz aditiva) para HSV (matiz, saturação, valor)
- **Por que não usar BGR direto:** uma linha "preta" em BGR pode ser
  `(40, 40, 40)`, `(20, 30, 50)` (sombra), `(10, 10, 10)` (escuro
  forte) — não dá pra escrever um threshold simples sem confundir
  com outras coisas escuras
- Em HSV, "preto" = **V (valor) baixo**, independente de H e S. Um
  único limite em V isola tudo que é escuro

Convenções OpenCV para HSV:
- H: 0–180 (não 0–360!)
- S: 0–255
- V: 0–255

---

### Passo 4 — Filtro mediano

```python
hsv = cv2.medianBlur(hsv, 9)
```

- Filtro mediana 9×9: para cada pixel, substitui pelo valor mediano
  da janela 9×9 ao redor
- **Por que mediana e não Gaussiano:** mediana mata **outliers
  pontuais** (pixels brancos isolados em borda de mesh, jitter de
  shader) preservando arestas. Gaussiano borraria as arestas da linha
- Tamanho 9 é grande o bastante pra remover ruído de até ~4 pixels
  sem comer detalhe estrutural da linha

---

### Passo 5 — Segmentação (threshold)

```python
mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 65]))
```

- `inRange` retorna uma máscara binária `uint8` (0 ou 255) de mesmo
  shape espacial da entrada — `(96, 320)`
- Critério: pixel está na máscara se
  - `0 ≤ H ≤ 180` (qualquer matiz)
  - `0 ≤ S ≤ 255` (qualquer saturação)
  - `0 ≤ V ≤ 65` (somente **escuro**)
- O limite superior de V (65) é o knob mais importante para a
  qualidade da segmentação. Aumentar pega mais cinza claro (linha em
  baixa iluminação); diminuir descarta sombras

---

### Passo 6 — Encontrar contornos

```python
contours, _ = cv2.findContours(
    mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
```

- `findContours` extrai curvas fechadas da máscara binária
- `RETR_EXTERNAL`: só os contornos **externos** (ignora buracos
  internos) — basta para a linha
- `CHAIN_APPROX_NONE`: retorna **todos** os pontos do contorno (sem
  reduzir colineares). Custa um pouco mais de memória mas o cálculo
  do centróide via momentos não usa essa estrutura ponto-a-ponto
- `contours` é uma lista de arrays; cada array é um contorno

---

### Passo 7 — Maior contorno

```python
c = max(contours, key=cv2.contourArea)
```

- Escolhe o contorno de maior **área** como sendo "a linha"
- Filtra naturalmente blobs ruidosos pequenos (uma sombra solta de 20
  pixels não vai ganhar do contorno da linha que tem milhares)
- **Falha conhecida:** se há duas linhas igualmente visíveis
  (intersecção em T), pode pular entre elas

---

### Passo 8 — Centróide via momentos

```python
M = cv2.moments(c)
if M["m00"] == 0:
    # contorno degenerado (área zero) — trata como sem linha
    self.bc.move(0.0, 0.0)
    return
cx = int(M["m10"] / M["m00"])
cy = int(M["m01"] / M["m00"])
```

- Momentos de imagem: `m00` é a área, `m10` é a soma de `x` sobre os
  pixels, `m01` é a soma de `y`
- Centróide = `(m10/m00, m01/m00)` é o **ponto médio** do contorno
  no plano da imagem
- A guarda `m00 == 0` evita divisão por zero — contornos podem ter
  área zero em casos degenerados (linha de um pixel)

---

### Passo 9 — Erro horizontal

```python
h_roi, w_roi = mask.shape
target_x = w_roi // 2          # = 160, centro horizontal da ROI

error_x = cx - target_x
```

- `target_x` = onde a linha **deveria estar** para o robô seguir reto
  (centro horizontal da imagem)
- `error_x`:
  - **negativo** → `cx < target_x` → linha à esquerda do centro
  - **positivo** → `cx > target_x` → linha à direita
  - **zero** → centróide está exatamente no centro (perfeito)

---

### Passo 10 — Deadband + comando angular

```python
if abs(error_x) < DEADBAND_PX:   # DEADBAND_PX = 5
    angular = 0.0
else:
    angular = -K_ANGULAR * error_x   # K_ANGULAR = 0.006
```

**Deadband:** se o erro é menor que 5 pixels, considera "em cima da
linha" e não comanda rotação. Sem isso, o ruído de quantização da
imagem (centróide oscila ±1–2 pixels mesmo parado) gera comando
angular constante e o robô tremelica.

**Sinal negativo (importante!):** a convenção do ROS para `angular.z`
é:

| `angular.z` | Sentido | Direção do giro |
|---|---|---|
| `> 0` | anti-horário (visto de cima) | **esquerda** |
| `< 0` | horário | **direita** |
| `= 0` | sem rotação | reto |

Queremos:
- Linha à esquerda (`error_x < 0`) → virar para a esquerda → `angular > 0`
- Linha à direita (`error_x > 0`) → virar para a direita → `angular < 0`

A relação que satisfaz ambos os casos é `angular = -K · error_x`.
Sem o sinal negativo, o robô viraria **para longe da linha** e
divergiria imediatamente.

**Magnitude:** `K_ANGULAR = 0.006` rad/s/px. No erro máximo
(half-width = 160 px), gera `0.96 rad/s` ≈ 55°/s — o suficiente para
curvar sem instabilidade.

---

### Passo 11 — Velocidade linear adaptativa

```python
t = min(abs(error_x) / ERROR_FULL_CURVE, 1.0)
linear = LINEAR_STRAIGHT * (1.0 - t) + LINEAR_CURVE * t
```

Interpolação linear (lerp) entre duas velocidades:

- `LINEAR_STRAIGHT = 0.14 m/s` — velocidade quando `error_x = 0`
- `LINEAR_CURVE = 0.05 m/s` — velocidade quando `|error_x| ≥ 60 px`
- Entre 0 e 60 px de erro, a velocidade desce **linearmente** de 0.14
  para 0.05

```
linear (m/s)
 0.14 ┤●─────╮
      │      ╲
 0.10 ┤       ╲
      │        ╲
 0.05 ┤         ╲──────●─────
      └─┬────┬────┬────┬─────
        0   20   40   60  ≥60
                                |error_x| (px)
```

**Intuição:** quanto mais a linha está fora do centro, mais agressiva
precisa ser a curva — e curvas em alta velocidade levam o robô para
fora da pista por inércia. Desacelerar **antes** de entrar na curva
(velocidade adaptativa ao erro) é a forma mais simples de evitar
overshoot sem PID completo.

---

### Passo 12 — Despachar o comando

```python
self.bc.move(linear, angular)
```

`bot_control.move` empacota como `TwistStamped` e publica em `/cmd_vel`.
Detalhes na próxima seção.

---

### Passo 13 — Caso sem linha

```python
if not contours:
    self.bc.move(0.0, 0.0)
    self._publish_debug(roi, mask, None, target_x)
    return
```

Quando nenhum contorno é encontrado (linha fora do campo de visão,
máscara vazia), o robô **para** (`linear=0, angular=0`).

**Alternativas possíveis** (não implementadas):
- Manter o último comando (continua andando "no escuro")
- Girar no lugar procurando a linha (`linear=0, angular=±k`)
- Voltar para trás (`linear<0`)

O "parar" é o mais seguro como default; outras estratégias dependem
do tipo de pista.

---

### Passo 14 — Debug image (paralelo, opcional)

```python
def _publish_debug(self, roi, mask, centroid, target_x):
    if self.debug_pub.get_subscription_count() == 0:
        return
    debug = roi.copy()
    debug[mask > 0] = (0, 255, 0)             # máscara em verde
    cv2.line(debug, (target_x, 0), (target_x, debug.shape[0]),
             (255, 0, 0), 1)                  # linha-alvo em azul
    if centroid is not None:
        cx, cy, contour = centroid
        cv2.drawContours(debug, [contour], -1, (0, 255, 255), 1)  # contorno amarelo
        cv2.circle(debug, (cx, cy), 4, (0, 0, 255), -1)            # centróide vermelho
    msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
    self.debug_pub.publish(msg)
```

- Compõe um frame visual que mostra **o que o algoritmo está
  decidindo**: o que ele segmentou (verde), onde quer chegar (azul),
  o que detectou como linha (amarelo), e o centróide computado
  (vermelho)
- **Otimização chave:** `get_subscription_count() == 0` faz a função
  retornar imediatamente quando ninguém está escutando o tópico —
  zero overhead quando o `viewer` está fechado
- O frame é publicado em `/line_follower/debug_image` — o `view.py`
  ou um `rqt_image_view` qualquer pode visualizar

---

## `robot_controller.py` — empacotar e publicar

A classe `bot_control` esconde dois detalhes:

1. Que `/cmd_vel` espera **`TwistStamped`** (não `Twist`) porque é
   isso que o `parameter_bridge` espera no ros→gz para o
   `turtlebot3_burger_cam`
2. O `header.stamp` precisa ser atualizado **a cada publish** (se for
   constante, `use_sim_time` reclama de timestamps repetidos)

### Construção ([robot_controller.py:8-17](scripts/robot_controller.py#L8-L17))

```python
class bot_control:
    def __init__(self, node):
        self.node = node
        self.velocity_msg = TwistStamped()
        self.velocity_msg.header.frame_id = 'base_footprint'
        self.velocity_msg.twist.linear.y = 0.0
        ...
        self.pub = self.node.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.P = 0.004
```

- Recebe o **node** do `LineFollower` e cria o publisher **nesse node**
  (não cria um node próprio — economiza um nó na grafo)
- Pré-zera as componentes não usadas do twist (linear.y, linear.z,
  angular.x, angular.y) uma vez só. Em cada `move`, só precisará
  setar `linear.x` e `angular.z`
- `frame_id = 'base_footprint'` — convenção: comandos são no frame
  do robô no chão

### Publicar ([robot_controller.py:20-24](scripts/robot_controller.py#L20-L24))

```python
def move(self, linear, angular):
    self.velocity_msg.header.stamp = self.node.get_clock().now().to_msg()
    self.velocity_msg.twist.linear.x = float(linear)
    self.velocity_msg.twist.angular.z = float(angular)
    self.pub.publish(self.velocity_msg)
```

- `header.stamp` é renovado a cada chamada com o relógio atual
  (relógio sim se `use_sim_time=True`)
- `linear.x` ⇒ avanço/recuo (m/s)
- `angular.z` ⇒ rotação no plano xy (rad/s)
- `publish` envia para `/cmd_vel`

A partir daí, o `parameter_bridge` traduz para `gz.msgs.Twist`,
o plugin `gz-sim-diff-drive-system` recebe e calcula as velocidades
de roda esquerda/direita, e o robô se move.

### `fix_error` (legado, não usado pelo line2)

A classe ainda tem um método `fix_error(linear_error, orien_error)` que
existe para outros scripts antigos. **Não é chamado pelo
`line2.py`** — o `LineFollower` invoca `move` diretamente porque
`fix_error` tem um bias `+0.6` na branch `orien_error < 0` que faz o
robô virar sempre para a esquerda. Bypass intencional.

---

## `view.py` — visualizar o que o robô vê

O `viewer` é um **processo separado** do `line_follower` por um motivo
crítico: chamadas `cv2.imshow` / `cv2.waitKey` precisam rodar na
**main thread** do processo. Se forem chamadas dentro de um callback
ROS (que roda na thread do executor), no shutdown o `ros2 launch`
escala SIGTERM → SIGKILL e o processo morre com `exit -9`.

### Subscriptions ([view.py:23-32](scripts/view.py#L23-L32))

```python
class Viewer(Node):
    def __init__(self):
        super().__init__('line_follower_viewer')
        self.bridge = cv_bridge.CvBridge()
        self.raw = None
        self.debug = None
        self.create_subscription(Image, RAW_TOPIC, self._on_raw, 10)
        self.create_subscription(Image, DEBUG_TOPIC, self._on_debug, 10)

    def _on_raw(self, msg):
        self.raw = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _on_debug(self, msg):
        self.debug = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
```

Cada callback só **armazena o último frame** — não desenha nada.
A pintura acontece fora dos callbacks.

### Render — side by side ([view.py:34-50](scripts/view.py#L34-L50))

```python
def render(self):
    if self.raw is None and self.debug is None:
        return None
    h, w = (self.raw.shape[:2] if self.raw is not None else (240, 320))
    left = self.raw if self.raw is not None else np.zeros((h, w, 3), dtype=np.uint8)
    right = np.zeros((h, w, 3), dtype=np.uint8)
    if self.debug is not None:
        dh, dw = self.debug.shape[:2]
        right[h-dh:h, :dw] = self.debug[:dh, :dw]   # debug no fundo do canvas
    return np.hstack([left, right])
```

- **Esquerda:** frame raw (240×320)
- **Direita:** canvas 240×320 preto com o debug image (96×320) colado
  no fundo. Por que no fundo? Porque o debug é a ROI inferior — colado
  no fundo ele fica alinhado verticalmente com a parte equivalente do
  frame raw à esquerda
- `hstack` resulta em um canvas 240×640 BGR pronto pro `imshow`

### Main loop ([view.py:53-71](scripts/view.py#L53-L71))

```python
def main(args=None):
    rclpy.init(args=args)
    node = Viewer()
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            frame = node.render()
            if frame is not None:
                cv2.imshow(WINDOW, frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.try_shutdown()
```

- `spin_once(timeout_sec=0.01)` processa **uma rodada** de callbacks
  e volta em até 10 ms — alimenta `self.raw` e `self.debug`
- `render()` monta o frame composto
- `cv2.imshow` + `cv2.waitKey(1)` — pintura ocorre **na main thread**
  do processo. Sem perigo de bloqueio durante shutdown
- Tecla `q` sai graceful

---

## Loop completo (resumo)

A cada frame da câmera (~30 Hz):

```
1.  msg ROS Image em /camera/image_raw           ← bridge gz→ROS
2.  cv_bridge.imgmsg_to_cv2 → np.ndarray BGR (240×320×3)
3.  _crop_bottom → ROI 40% inferior (96×320×3)
4.  cvtColor BGR→HSV + medianBlur (9×9)
5.  inRange (V ≤ 65) → máscara binária (96×320)
6.  findContours → lista de contornos externos
7.  if vazio → bc.move(0,0); return
8.  max(contours, key=contourArea) → maior blob
9.  moments → centróide (cx, cy)
10. error_x = cx - 160
11. angular = -0.006 * error_x  (com deadband ±5px)
12. linear  = lerp(0.14, 0.05, |error_x|/60)
13. bc.move(linear, angular)
    └─► robot_controller.move
        ├─ TwistStamped.header.stamp = now()
        ├─ twist.linear.x = linear; twist.angular.z = angular
        └─ pub.publish → /cmd_vel ─► bridge ─► gz diff_drive ─► rodas
14. _publish_debug → /line_follower/debug_image (se houver subscriber)
```

Em paralelo, no processo do `viewer`:

```
loop while rclpy.ok():
   spin_once(0.01s)           ← recebe raw + debug
   render()                   ← compõe canvas 240×640
   cv2.imshow / waitKey(1)    ← pinta a janela na main thread
```
