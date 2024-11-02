# RMI 

## Nome dos participantes

|  Num Mec | Nome             |
| ----: | ------------------- | 
| 118485|   Gustavo Reggio    |   
| 102959|   Pedro Maia        |  



# Assignment 1

## Instruções para aceder o repositório:
```
 cd ~ /RMI
 git clone https://github.com/iris-ua/ciberRatoTools.git
 ```

## Chalange 1 - Control 
Nesta fase, o agente deve ser capaz de navegar de forma autônoma pelo circuito usando sensores de distância para evitar colisões e decidir o próximo movimento. O objetivo é percorrer o maior número possível de voltas dentro de um tempo limitado.
```
cd ~/RMI/RMI
```
```
./startC1
```
## Chalange 2 - Mapping
Com um sensor GPS sem ruído, o agente deve explorar o labirinto, criando um mapa usando os sensores de distância. Além disso, o agente deve identificar e seguir caminhos ainda não explorados.
```
./startC2
```
## Chalange 3 - Planning
Nesta fase, o agente deve localizar pontos-alvo no labirinto e calcular o caminho mais curto que passa por todos eles, retornando ao ponto de partida.
```
./startC3
```
