# Estação simulada sobre alerta de chuvas e nível de rios 
Repositório criado para versionamento da atividade sobre FreeRTOS (multitarefas) da residência em software embarcado, com implementação de uma estação simualada de enchentes.


## Descrição geral do Funcionamento do programa 
No Display da BitDogLab é informado os níveis atuais de chuva e volume dos rios, como também o status atual (normal ou alto). O eixo X do joystick simula o volume dos rios e o  eixo Y do joystick simula o volume de chuva (ambos percentuais mostrados em tempo real). Quando os volumes atingem um nível crítico (rios>= 70% e chuvas>=80%), é informado estado de alerta no display, sinais sonoros distintos e informações de perigo nos LEDs ( símbolo de X na matriz de LEDs). 
## Descrição detalhada do Funcionamento do programa  na BitDogLab
Botões→ O botão B coloca a placa em modo Bootsel.
LED RGB→LED  azul GPIO 12, acionado quando volume dos rios é >= 70%. LED verde GPIO 11, acionado quando volume das chuvas é >= 80% .LED vermelho GPIO 13, acionado quando as duas condições são verdadeiras simultaneamente. 
Joystick→ Conversores ADC que simulam os sensores, com o  eixo X simulando o volume dos rios e o eixo Y simulando o volume de chuva. 
Display→ Mostra informações dos sensores simulados em tempo real, e outras informações sobre alerta crítico e níveis normais ou altos. 
Matriz de LEDs 5X5→ GPIO 7 que representa sinal de perigo (X) na cor azul quando volume dos rios é >= 70%. Representa sinal de perigo (X) na cor verde quando volume de chuva é >= 80%. Representa sinal de perigo (X) na cor vermelha de forma pulsante quando ambas as condições são verdadeiras.
Buzzer→ Buzzer GPIO 21 com sinal sonoro pulsante a cada 500 ms para volume dos rios >= 70%. Sinal sonoro pulsante a cada 1 s para volume de chuva  >= 80%. Sinal sonoro pulsante a cada 100 ms quando ambas as condições são verdadeiras (perigo eminente máximo). 

## Compilação e Execução
**OBS: É preciso ter o FreeRTOS baixado no computador, sendo necessário alterar a linha  do CMakeLists.txt com o diretório correspondente ao arquivo**


1. Certifique-se de que o SDK do Raspberry Pi Pico está configurado no seu ambiente.
2. Compile o programa utilizando a extensão **Raspberry Pi Pico Project** no VS Code:
   - Abra o projeto no VS Code, na pasta PRPJETO_MULTIRAREFAS tem os arquivos necessários para importar 
   o projeto com a extensão **Raspberry Pi Pico Project**.
   - Vá até a extensão do **Raspberry pi pico project** e após importar (escolher sdk de sua escolha) os projetos  clique em **Compile Project**.
3. Coloque a placa em modo BOOTSEL e copie o arquivo `AlertaEnchente.uf2`  que está na pasta build, para a BitDogLab conectado via USB.

**OBS: Devem importar o projeto para gerar a pasta build, pois a mesma não foi inserida no repositório**

## Colaboradores
- [PauloCesar53 - Paulo César de Jesus Di Lauro ] (https://github.com/PauloCesar53)
