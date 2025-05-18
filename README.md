# User Story BreakDown (SCOMP 24/25) - 2DI

## Diagrama de Componentes

<pre lang="markdown">
+---------------------------------------------+
|                Main Process                 |
|                 (main.c)                    |-----------+
+-----+----------------+----------------+-----+           |
      |                |                |                 |
      v                v                v                 v
+-------------+ +-------------+ +----------------+ +-------------+
| Initialize  | |    Start    | |   Generate     | |   Cleanup   |
| Simulation  | | Simulation  | |   Report       | | Simulation  |
+------+------+ +------+------+ +-------+--------+ +------+------+
       |               |                |                |
       v               v                v                v
+-------------+ +-------------+ +----------------+ +-------------+
| Read Drone's| | Fork        | | Write          | | Send        |
| File        | | Processes   | | Simulation     | | Termination |
+------+------+ +------+------+ | Report         | | Signals     |
       |               |        +----------------+ +------+------+
       v               |                                  |
+-------------+        |                                  v
| Create Drone|        |                           +-------------+
| Structures  |        |                           | Wait for    |
+------+------+        |                           | Processes   |
       |               |                           +------+------+
       v               |                                  |
+-------------+        |                                  v
| Create Pipes|        |                           +-------------+
+-------------+        |                           | Close Pipes |
                       |                           +-------------+
                       v
       +---------------+---------------+
       |                               |
       v                               v
+-------------+                +----------------+
| Parent      |<--Pipes------->| Child Processes|
| Process     |<--Signals----->| (Drones)       |
+------+------+                +-------+--------+
       |                               |
       v                               |
+-------------+                        |
| Read        |                        |
| Positions   |                        |
+------+------+                        |
       |                               |
       v                               |
+-------------+                        |
| Check       |                        |
| Collisions  |                        |
+------+------+                        |
       |                               |
       v                               |
+-------------+                        |
| Terminate   |                        |
| Colliding   |                        |
| Drones      |                        |
+-------------+                        |
                                       |
                                       v
                       +---------------+---------------+
                       |               |               |
                       v               v               v
               +-------------+ +-------------+ +-------------+
               | Drone       | | Drone       | | Drone       |
               | Process 0   | | Process 1   | | Process N   |
               +------+------+ +------+------+ +------+------+
                      |               |               |
                      v               v               v
               +-------------+ +-------------+ +-------------+
               | Read        | | Read        | | Read        |
               | Script 0    | | Script 1    | | Script N    |
               +-------------+ +-------------+ +-------------+
</pre>

## Exemplo de Script de Movimento

Conteúdo do 'drone_0_script.txt':

| Tempo (em segundos) |  X  |  Y  |  Z  |
|:-------------------:|:---:|:---:|:---:|
|         1.0         | 5.0 | 5.0 | 5.0 |
|         2.0         | 4.5 | 4.5 | 4.5 |
|         3.0         | 4.0 | 4.0 | 4.0 | 
|         4.0         | 3.5 | 4.5 | 3.5 |
|         5.0         | 3.0 | 3.0 | 3.0 |
|         6.0         | 2.5 | 2.5 | 2.5 |
|         7.0         | 2.0 | 2.0 | 2.0 |
|         8.0         | 1.5 | 1.5 | 1.5 |
|         9.0         | 1.0 | 1.0 | 1.0 |
|        10.0         | 0.5 | 0.5 | 0.5 |


## Abordagem e Implementação

#### US261 - Initiate simulation for a figure

- Ler os scripts dos drones selecionados e de seguida escolher o raio pretendido .
- Criar um processo filho para cada drone utilizando a função fork().
- Cada drone executa os movimentos definidos no seu script específico.
- Comunicação com o processo pai via pipe().

#### US262 - Capture and process drone movements

- Cada drone envia as coordenadas da sua posição para o processo pai através do write().
- O processo pai lê as informações com read() e atualiza um array que mantém o estado dos drones.
- Cada entrada do array contém o timestamp, as coordenadas (X, Y, Z) e o ID do drone.


#### US263 - Detect drone collisions in real time

- A cada atualização de posição, o sistema calcula a distância entre os drones.
- Se a distância entre dois drones for inferior à definida pelo utilizador, uma colisão é registada.
- Os drones envolvidos recebem um sinal SIGSUP.
- O sistema armazena o timestamp e o ID dos drones envolvidos na colisão.


#### US264 - Synchronize drone execution with a time step

- A simulação avança com passos de tempo de 100ms, utilizando a função usleep(100000).
- Cada drone lê uma linha do seu script e espera o tempo correspondente antes de executar o movimento.
- A sincronização é garantida pela temporização dos rocessos.



#### US265 - Generate a simulation report

- Criação do ficheiro "Simulation_Report.txt" com os seguintes atributos:

  -> Resumo: 
      - Número total de drones
      - Número total de colisões 
      - Número máximo de colisões para ocrroer a falha 
      - Estado final (executado ou colisão)

  -> INFORMAÇÕES DO DRONE: 
      - ID do drone
      - Script usado
      - Status (sucesso ou falha)
      - Posição final (X, Y, Z)

  -> DETALHES DA COLISÃO:
    - ID dos drones envolvidos
    - Tempo de colisão
    - Timestamp da colisão do primeiro drone
    - Timestamp da colisão do segundo drone

  -> RECOMENDAÇÕES


## Auto-avaliação de compromisso

|        Nome        | Compromisso (%) | Auto-avaliação | 
|:------------------:|:---------------:|:--------------:|
|    Ana Oliveira    |       85        |       15       | 
|   Beatriz Costa    |       85        |       15       | 
|   António Teles    |       85        |       15       |  
| Francisco Silveira |       85        |       15       |
