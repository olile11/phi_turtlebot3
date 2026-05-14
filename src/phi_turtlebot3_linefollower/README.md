# phi_turtlebot3_linefollower

Line follower baseado em câmera para o TurtleBot3 burger em Gazebo Sim
(Harmonic) + ROS 2 Jazzy. Captura imagem RGB, segmenta o pixel "preto"
da linha em HSV, calcula o centróide do maior contorno e fecha o loop
de controle por erro horizontal entre o centróide e o centro da imagem.

---

## Quick start

```bash
# Da raiz do workspace
colcon build --packages-select phi_turtlebot3_linefollower phi_turtlebot3_description
source install/setup.bash
ros2 launch phi_turtlebot3_linefollower simulation.launch.py
```

Se um `gz sim` órfão de uma execução anterior estiver vivo, mate antes:

```bash
pkill -9 -f "gz sim"
```

---

## Visão geral do fluxo

```
┌─────────────────────┐  gz topic   ┌──────────────────┐  ROS topic   ┌─────────────────┐
│ Gazebo Sim          │ camera/     │ parameter_bridge │  /camera/    │ line_follower   │
│  turtlebot3_burger_ │ image_raw   │  (model_bridge.  │  image_raw   │  (scripts/      │
│  cam (SDF)          │────────────▶│   yaml)          │─────────────▶│   line2.py)     │
│                     │             │                  │              │                 │
│  diff_drive plugin  │◀────────────│                  │◀─────────────│  bot_control    │
│                     │  cmd_vel    │                  │  /cmd_vel    │  (robot_        │
│                     │  (gz Twist) │                  │  TwistStamped│   controller.py)│
└─────────────────────┘             └──────────────────┘              └─────────────────┘
                                                                              │
                                                                              │ /line_follower/
                                                                              │ debug_image
                                                                              ▼
                                                                      ┌──────────────────┐
                                                                      │ viewer           │
                                                                      │ (scripts/        │
                                                                      │  view.py)        │
                                                                      │  janela cv2      │
                                                                      └──────────────────┘
```

---

## Estrutura de arquivos

```
phi_turtlebot3_linefollower/
├── CMakeLists.txt           # Build/install: registra scripts e config como executáveis e share/
├── package.xml              # Dependências ROS (rclpy, cv_bridge, ros_gz_bridge, rqt_image_view, …)
├── README.md                # Este arquivo
│
├── config/
│   ├── model_bridge.yaml    # Mapeamento ros↔gz de TODOS os tópicos (clock, cmd_vel, image, scan, …)
│   └── line_follow.yaml     # (parâmetros de tuning, opcional — não usado no código atual)
│
├── launch/
│   ├── simulation.launch.py # Launch principal: gz sim + spawn + bridge + line_follower + viewer
│   └── line_follower.launch.py  # Só o nó de controle (útil em hardware real)
│
└── scripts/
    ├── line2.py             # Nó principal (executável: `line_follower`)
    ├── view.py              # Visualizador cv2 (executável: `viewer`)
    ├── robot_controller.py  # Classe bot_control: publica TwistStamped em /cmd_vel
    └── Line_follow.py       # Speed controller standalone (legado, não usado no fluxo principal)
```

### Como cada arquivo se conecta

| Arquivo | Papel | Quem chama / quem é chamado |
|---|---|---|
| `simulation.launch.py` | Orquestra a simulação | Inclui `spawn_gazebo.launch.py`, `spawn_states_publishers.launch.py` da `phi_turtlebot3_description`, faz spawn do robô com `ros_gz_sim create`, sobe o `parameter_bridge` lendo `config/model_bridge.yaml`, inclui `line_follower.launch.py` e `viewer` |
| `line_follower.launch.py` | Sobe só o nó de controle | Roda o executável `line_follower` (que é `scripts/line2.py`) |
| `model_bridge.yaml` | Define a tradução ros↔gz | Lido pelo `parameter_bridge` no `simulation.launch.py` |
| `scripts/line2.py` | Pipeline câmera→controle | Subscribe `/camera/image_raw`, instancia `bot_control(self)`, chama `bot_control.move(linear, angular)`, publica em `/line_follower/debug_image` |
| `scripts/robot_controller.py` | Helper de publicação | Importado por `line2.py`. Cria publisher TwistStamped em `/cmd_vel` e expõe `move(linear, angular)` |
| `scripts/view.py` | Visualização cv2 | Subscribe `/camera/image_raw` e `/line_follower/debug_image`, monta side-by-side, mostra com `cv2.imshow` na main thread |

### Dependência externa: phi_turtlebot3_description

Este pacote **não tem** o robô nem o world. Eles vivem em `phi_turtlebot3_description`:

- `models/turtlebot3_burger_cam/model.sdf` — SDF do robô com a câmera (Intel D435 visual + sensor pinhole 320x240, 80° FOV) e plugin `gz-sim-diff-drive-system`
- `urdf/turtlebot3_burger_cam.urdf` — URDF para `robot_state_publisher` (TF), com `camera_link` (body frame) e `camera_rgb_frame` (optical frame)
- `worlds/line_world.sdf` — Mundo com plugins gz-sim (physics/sensors/IMU), ground plane e o modelo `black_line` plano no chão

---

## Pipeline do algoritmo (passo a passo)

### 1 — Aquisição da imagem
- Câmera no SDF: 320×240, BGR, 30 Hz, `<topic>camera/image_raw</topic>`
- Bridge yaml mapeia gz `camera/image_raw` ↔ ROS `/camera/image_raw` (tipo `sensor_msgs/msg/Image`)
- `line2.py:30` faz `create_subscription(Image, CAMERA_TOPIC, self.callback, 10)`
- `cv_bridge.imgmsg_to_cv2(data, "bgr8")` converte para `np.ndarray` BGR (shape 240×320×3)

### 2 — ROI (Region of Interest)
- `_crop_bottom(frame)` em [line2.py:39](scripts/line2.py#L39) corta os **40% inferiores** da imagem (parte mais próxima do robô)
- Por quê: a linha distante distorce o centróide e atrasa a reação; usar só a faixa próxima dá reação local mais limpa
- Resultado: 96×320×3

### 3 — Segmentação HSV
- `cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)` — converte para HSV (matiz / saturação / valor)
- `cv2.medianBlur(hsv, 9)` — filtro mediana 9×9 para tirar ruído pontual
- `cv2.inRange(hsv, [0,0,0], [180,255,65])` — máscara binária de pixels **escuros**: V (valor) ≤ 65 ⇒ "preto"
- Por que HSV e não BGR: o canal V isola luminância. Ground plane tem V alto (claro), linha tem V baixo (escuro) — separação robusta independente da tonalidade

### 4 — Encontrar a linha
- `cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)` — extrai contornos externos da máscara binária
- `max(contours, key=cv2.contourArea)` — pega o maior contorno (filtra blobs ruidosos pequenos)
- `cv2.moments(c)` + `cx = M["m10"]/M["m00"]`, `cy = M["m01"]/M["m00"]` — centróide

### 5 — Erro horizontal
- `target_x = w_roi // 2` = 160 (centro da ROI; onde a linha deveria estar pro robô seguir reto)
- `error_x = cx - target_x`
  - `error_x < 0` ⇒ linha à **esquerda** do centro → robô deve virar pra esquerda
  - `error_x > 0` ⇒ linha à **direita** → virar pra direita
  - `|error_x| < DEADBAND_PX (5)` ⇒ está em cima da linha, segue reto

### 6 — Comando angular (proporcional)
- `angular = -K_ANGULAR * error_x` em [line2.py:81](scripts/line2.py#L81)
- O **sinal negativo** é o coração do controle: convenção ROS é `angular.z > 0` = giro anti-horário = vira **esquerda**. Como `error_x < 0` significa "linha à esquerda" e queremos virar pra esquerda, `angular.z = -K * (negativo) = positivo` ⇒ vira pra esquerda ✓
- `K_ANGULAR = 0.006` rad/s por pixel ⇒ em erro máximo (160 px) dá ~0.96 rad/s

### 7 — Comando linear (adaptativo)
- Interpolação linear entre `LINEAR_STRAIGHT` (rápido na reta) e `LINEAR_CURVE` (devagar na curva):
  ```
  t = min(|error_x| / ERROR_FULL_CURVE, 1.0)
  linear = LINEAR_STRAIGHT * (1 - t) + LINEAR_CURVE * t
  ```
- `error_x = 0` ⇒ `t = 0` ⇒ `linear = LINEAR_STRAIGHT = 0.14` m/s
- `|error_x| ≥ 60` ⇒ `t = 1` ⇒ `linear = LINEAR_CURVE = 0.05` m/s
- No meio, transição linear → o robô **desacelera suavemente** quando entra em curva

### 8 — Publicação do comando
- `bot_control.move(linear, angular)` em [robot_controller.py:20](scripts/robot_controller.py#L20)
- Monta `TwistStamped`: `header.stamp = now()`, `header.frame_id = base_footprint`, `twist.linear.x = linear`, `twist.angular.z = angular`
- Publica em `/cmd_vel` (tipo `geometry_msgs/msg/TwistStamped`)

### 9 — Bridge ros → gz
- `parameter_bridge` (sobe pelo launch lendo `model_bridge.yaml`) traduz:
  ```yaml
  - ros_topic_name: "cmd_vel"
    gz_topic_name:  "cmd_vel"
    ros_type_name:  "geometry_msgs/msg/TwistStamped"
    gz_type_name:   "gz.msgs.Twist"
    direction:      ROS_TO_GZ
  ```
- Stamped (ROS) → não-stamped (gz Twist): o bridge descarta o header e converte o twist interno

### 10 — Atuação no robô
- Plugin `gz-sim-diff-drive-system` no SDF subscribe gz `cmd_vel` e calcula velocidades de roda usando `wheel_separation=0.160`, `wheel_radius=0.033`
- Aplica as velocidades nos joints `wheel_left_joint` e `wheel_right_joint`

### 11 — Caso sem linha
- `if not contours` ou `M["m00"] == 0` ⇒ `bot_control.move(0.0, 0.0)` (parado)
- Importante: o `fix_error(0,0)` legado caía em `move(0.4, 0.0)`, o que disparava o robô pra fora da pista quando perdia a linha. Por isso `line2.py` chama `move` direto

### 12 — Debug image (paralelo, opcional)
- `_publish_debug` em [line2.py:94](scripts/line2.py#L94) compõe um frame BGR com:
  - ROI original (fundo)
  - Máscara em **verde** (`debug[mask>0] = (0,255,0)`)
  - Linha-alvo em **azul** vertical em `target_x`
  - Contorno do maior blob em **amarelo**
  - Centróide em **vermelho**
- Publica em `/line_follower/debug_image`
- Optimização: se `get_subscription_count() == 0`, retorna imediatamente — não custa nada quando o `viewer` está fechado

### 13 — Viewer
- `view.py` subscribe `/camera/image_raw` (raw) e `/line_follower/debug_image` (anotado)
- A cada iteração do loop principal: monta canvas 640×240 com **raw à esquerda, debug à direita**, chama `cv2.imshow`
- **Crucial:** `cv2.imshow` e `cv2.waitKey` rodam **só na main thread** (não em callback ROS). É por isso que esse viewer é um processo separado — `cv2.imshow` no callback do `line_follower` travava o executor e gerava SIGKILL no shutdown
- Tecla `q` na janela fecha graceful

---

## Tabela de tópicos

| Tópico ROS | Tipo | Direção | Tópico GZ | Quem publica | Quem subscribe |
|---|---|---|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | gz→ros | `clock` | gz sim | todos com `use_sim_time` |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | gz→ros | `camera/image_raw` | sensor da câmera | `line_follower`, `viewer` |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | gz→ros | `camera/camera_info` | sensor da câmera | (futuro: calibração) |
| `/scan` | `sensor_msgs/msg/LaserScan` | gz→ros | `scan` | sensor LDS | — |
| `/imu` | `sensor_msgs/msg/Imu` | gz→ros | `imu` | sensor IMU | — |
| `/odom` | `nav_msgs/msg/Odometry` | gz→ros | `odom` | diff_drive | (futuro: localização) |
| `/joint_states` | `sensor_msgs/msg/JointState` | gz→ros | `joint_states` | joint_state_publisher | `robot_state_publisher` |
| `/tf` | `tf2_msgs/msg/TFMessage` | gz→ros | `tf` | diff_drive | TF tree |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | ros→gz | `cmd_vel` | `bot_control` | diff_drive plugin |
| `/line_follower/debug_image` | `sensor_msgs/msg/Image` | (só ROS) | — | `line_follower` | `viewer` |

---

## Frames de coordenadas

Dois frames padrão para a câmera, ambos definidos no [URDF](../phi_turtlebot3_description/urdf/turtlebot3_burger_cam.urdf):

**`camera_link` — body frame (estrutura do robô)**
- X = forward, Y = left, Z = up
- Sensor SDF montado neste link, captura ao longo de +X
- Visual D435 sem rotação — lente aponta +X

**`camera_rgb_frame` — optical frame (algoritmos de visão / OpenCV)**
- X = right, Y = down, Z = forward (dentro da cena)
- Definido como filho de `camera_link` com `rpy="-1.5708 0 -1.5708"` (transform canônico body→optical)
- O SDF tem `<gz_frame_id>camera_rgb_frame</gz_frame_id>` → as `Image`/`CameraInfo` saem com `header.frame_id=camera_rgb_frame`, então projeção e cálculos em OpenCV funcionam direto sem reinterpretar eixos

---

## Parâmetros de ajuste

No topo de [line2.py](scripts/line2.py#L18):

| Parâmetro | Valor | Efeito |
|---|---|---|
| `LINEAR_STRAIGHT` | 0.14 m/s | Velocidade máxima na reta. Aumentar para correr mais; pode oscilar |
| `LINEAR_CURVE` | 0.05 m/s | Velocidade mínima em curva fechada. Diminuir se está saindo da curva |
| `ERROR_FULL_CURVE` | 60 px | Erro a partir do qual usa LINEAR_CURVE. Menor = desacelera mais cedo |
| `K_ANGULAR` | 0.006 rad/s/px | Ganho proporcional. Maior = vira mais rápido, mas oscila |
| `DEADBAND_PX` | 5 | Zona morta — abaixo disso ignora erro, segue reto |

Limiar da segmentação em [line2.py:51](scripts/line2.py#L51):
```python
cv2.inRange(hsv, [0,0,0], [180,255,65])
```
Aumentar o V (último valor) capta linhas mais claras; diminuir descarta sombras.

ROI em [line2.py:43](scripts/line2.py#L43): `frame[int(h*0.6):h, :]` — `0.6` controla o quanto pega da imagem. Menor (`0.4`) pega mais (vê mais à frente, antecipa curva); maior (`0.8`) pega menos (reação mais local).

---

## Troubleshooting

| Sintoma | Causa | Fix |
|---|---|---|
| `Requesting list of world names` (loop infinito) | 2+ instâncias de `gz sim` rodando, ambas com world `default` | `pkill -9 -f "gz sim"` antes do launch |
| `Found additional publishers on /clock /stats` | Idem acima | Idem |
| `line_follower-N process has died exit code -9` | SIGKILL — shutdown não conseguiu matar o nó (`cv2.imshow` em callback travava executor) | Já corrigido: `cv2.imshow` foi movido para `view.py` na main thread |
| Câmera tudo verde no debug | Threshold V=65 muito permissivo, OU iluminação ruim no world | Diminuir V em `cv2.inRange` |
| Robô vira sempre para o mesmo lado | Bug histórico em `bot_control.fix_error` (`+0.6` na branch < 0). `line2.py` agora chama `move` direto, ignora `fix_error` | — |
| Imagem rotacionada / câmera apontando errado | Mesh do d435 com rotação errada no `<visual>` | Conferir `<pose>` do visual no SDF do burger_cam |
| `not a valid package name` no launch | Variável recebendo path mas chamando `get_package_share_directory` de novo nela | Verificar nomes de variáveis em `simulation.launch.py` |
| Erro de build "failed to create symbolic link ... existing path cannot be removed" | Artefato antigo do `ament_cmake_python` | `rm -rf build/phi_turtlebot3_linefollower` e rebuild |

---

## Hardware real (sem Gazebo)

Para rodar no TurtleBot3 físico, use só [`line_follower.launch.py`](launch/line_follower.launch.py) com `use_sim_time:=false`:

```bash
ros2 launch phi_turtlebot3_linefollower line_follower.launch.py use_sim_time:=false
```

Notas:
- O TB3 real publica `Twist` (não `TwistStamped`) em `/cmd_vel`. Pra usar o nó no real, o `robot_controller.py` precisaria mudar o tipo de msg, ou usar um relay
- Câmera precisa estar publicando em `/camera/image_raw` (Raspberry Pi Cam via `v4l2_camera_node`, por exemplo)
- O ROI 40% inferior pode precisar de tuning dependendo da altura/ângulo da câmera real
