Biblioteca Loramesh
Author: Renato F. Fernandes, Natan Ferreira Alvarenga, Hermes G. Fernandes Neri, Gabriel Speroto
Descricao:  
   - Codigo lora utilizando placas Heltec WIFI ESP32 lora V2 e V3. 
    (para compilar para uma placa V2 ou V3 basta alterar o arquivo platformio.ini)
   - O codigo é baseado nas bibliotecas basicas da Heltec.
   https://github.com/HelTecAutomation/Heltec_ESP32/tree/master/examples
   e radiolib.h 

V1_01xx
   - Merge com a versao do Gabriel /Natan com tarefas de aplicacao e sendtask em arquivos separados.
   - esta versao nao funciona a resposta do escravo para o mestre na janela de tmepo especificada.
 

V1_01
   - Esta versão suporta V2 e V3, bastando alterar o platformio.ini. Esta funcionando as duas placas misturadas (ou seja, nao importa se eh V2 ou V3 elas conseguem enviar e receber para qualquer placa)
   - Foi implementado toda a parte de tratamento de dados no arquivo Loramesh
   - Agora nao eh necessario mais o define de ROUTER. Foi criado uma tabela com o DeviceID (serial number) de cada placa e enderecamento e tipo de devices fixos. 
     Importante!!! Deve ser incluido cada placa (serial number) na tabela devid (arquivo loramesh.cpp).
   - Recepcao é feia atraves de interrupcao. 
   - Apesar da criacao da tarefa App_task, foi utilizado o tratamento da maquina de estado ainda no loop (estava tendo problema para enviar e receber os frames).
   - testes de distancia:
   CONFIG1 : Freq=868.00 Bw=125.00 sf=7 cr=5 power=10 gain=0
   distancia maxima = 50m (janela da Maria até o portao do apartamento)
   CONFIG2 : Freq=868.00 Bw=125.00 sf=7 cr=5 power=20 gain=1
   distancia maxima = 50m (janela da Maria até o portao do apartamento)
   CONFIG3 : Freq=434.00 Bw=125.00 sf=9 cr=5 power=20 gain=1
   nao funcionou
   CONFIG4 : Freq=470.00 Bw=125.00 sf=9 cr=5 power=20 gain=1
   funcionou mas bem pertinho...com taxa de perda alta
   CONFIG5 : Freq=868.00 Bw=125.00 sf=9 cr=5 power=20 gain=1
   distancia maxima = 100m (janela da Maria até o meio da rua de baixo indo para a ana de godoy, no meio do quarteirao...ou seja 1 quarteirao e meio)
   CONFIG5 : Freq=868.00 Bw=125.00 sf=11 cr=5 power=20 gain=1
   distancia maxima = 150m
   
   melhorias:
   - testar se realmente o power esta funcionando em plena carga (talvez pegar o analisador de espectro da telecom)
   - a antena deve ser melhorada...
   - ver se tem influencia o radio antigo e o novo (V2 e V3)

V1_00
   -Esta versao agora consegue enviar e receber os pacotes de forma mais estavel...
   Ele nao esta usando tarefas ainda, mas ele consegue enviar e receber de uma placa End device. 
   - Esta versao nao usa interrupcao na recepcao.
   - Esta versão não suporta a placa V3 (somente a V2).

V0_00   
   -Inicio do codigo para rodar projeto loramesh (ainda em desenvolvimento)
   este codigo somente envia e recebe frames de diferentes tamanhos para 2 placas (ROUTER=1 envia e ROUTER=0 recebe).

   
