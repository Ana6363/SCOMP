
# User Story BreakDown (SCOMP 24/25) - 2DI

## Diagrama de Componentes

## Diagrama de Componentes

<pre lang="markdown">
+-------------------------------------------------------------+
|                        Main Process                         |
|                          (main.c)                           |
+--------------------+--------------------+-------------------+
                     |                    |
                     v                    v
          +-------------------+  +-------------------+
          | Collision Thread  |  | Report Thread     |
          +--------+----------+  +--------+----------+
                   |                      |
                   v                      v
          +--------+----------+  +--------+----------+
          | Scan SHM for      |  | Wait on Cond. Var |
          | collisions        |  | (pthread)         |
          +--------+----------+  +--------+----------+
                   |                      |
                   v                      v
          +--------+----------+  +--------+----------+
          | Signal Condition  |  | Process Report    |
          | Variable          |  | + Write Log       |
          +--------+----------+  +-------------------+
                   |
                   v
         +---------+-----------+
         | Shared Memory (SHM) |
         | + Mutex + Semaphores|
         +---------+-----------+
                   ^
                   |
  +----------------+---------------------------+
  |                |                           |
  v                v                           v
+----------------+  +----------------+  +----------------+
| Drone Process 0|  | Drone Process 1|  | Drone Process N|
+--------+-------+  +--------+-------+  +--------+-------+
         |                  |                   |
         v                  v                   v
+----------------+  +----------------+  +----------------+
| Read Script    |  | Read Script    |  | Read Script    |
| Update SHM     |  | Update SHM     |  | Update SHM     |
| Wait/Signal Sem|  | Wait/Signal Sem|  | Wait/Signal Sem|
+----------------+  +----------------+  +----------------+
</pre>
"""

## Exemplo de Script de Movimento

Conteúdo do 'drone_0_script.txt':

| Tempo (s) |    X    |    Y    |    Z    |
|:---------:|:-------:|:-------:|:-------:|
|   1.0     |  10.00  |  15.00  |  20.00  |
|   2.0     |  12.50  |  14.20  |  19.50  |
|   3.0     |  15.00  |  13.40  |  19.00  |
|   4.0     |  17.50  |  12.60  |  18.50  |
|   5.0     |  20.00  |  11.80  |  18.00  |
|   6.0     |  22.50  |  11.00  |  17.50  |
|   7.0     |  25.00  |  10.20  |  17.00  |
|   8.0     |  27.50  |   9.40  |  16.50  |
|   9.0     |  30.00  |   8.60  |  16.00  |
|  10.0     |  32.50  |   7.80  |  15.50  |
|  11.0     |  35.00  |   7.00  |  15.00  |
|  12.0     |  37.50  |   6.20  |  14.50  |
|  13.0     |  40.00  |   5.40  |  14.00  |
|  14.0     |  42.50  |   4.60  |  13.50  |
|  15.0     |  45.00  |   3.80  |  13.00  |
## Abordagem e Implementação

###  US361 - Inicializar simulação híbrida com memória partilhada

- Processo pai cria área de memória partilhada com `shmget`.
- Criação de processos drone com `fork()`, cada um acede à SHM.
- Sincronização entre processos via `sem_t` semáforos nomeados.
- Threads no processo pai iniciadas com `pthread_create`.

### US362 - Threads específicas para funções

- Thread `collision`: varre posições dos drones e deteta colisões.
- Thread `report`: aguarda sinalização por colisões e escreve log.
- Thread `step`: controla o avanço da simulação passo-a-passo, sinalizando os semáforos dos drones a cada "tick".


###  US363 - Notificação via condição

- `collision.c`: usa `pthread_cond_signal` ao detetar colisão.
- `report.c`: aguarda com `pthread_cond_wait`, depois escreve os dados.
- Proteção por `pthread_mutex_t`.

###  US364 - Sincronização passo-a-passo

- Cada drone espera no semáforo até permissão para o próximo passo.
- Pai sincroniza todos os semáforos no fim de cada tick.

###  US365 - Relatório final

- Relatório inclui:
    - Drones lançados
    - Posições finais
    - Colisões com timestamp
    - Resultado (sucesso/falha)
- Gerado por `report.c` e gravado em ficheiro.



## Auto-avaliação de compromisso

|        Nome        | Compromisso (%) | Auto-avaliação | 
|:------------------:|:---------------:|:--------------:|
|    Ana Oliveira    |       85        |       15       | 
|   Beatriz Costa    |       85        |       15       | 
|   António Teles    |       85        |       15       |  
| Francisco Silveira |       85        |       15       |
