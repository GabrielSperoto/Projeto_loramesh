# Projeto_loramesh
<p>O presente projeto tem como objetvio implementar uma rede mesh com base no protocolo lora. Atualmente, o código implementa uma rede de topologia estrela com três nós, sendo um deles o router (gateway). Pode ser dividido fundamentalmente em três tarefas:</p>
<ul>
  <l>Sendotask: responsável pelo envio do frame ou do beacon, no caso do router</l>
  <l>Aplicationtask: responsável por gerenciar a máquina de estados do sistem</l>
  <l>Watchdogtask: responsável por reiniciar a placa em caso de travamento p</l>
</ul>
<p>Além disso, o firmware faz a implementação a comunicação entre os dispositivos através de quatro janelas de tempo de 1000 ms (1s) cada. Sendo a janela 0 (slot 0) para o envio do beacon (router), e as janelas 2,3 para o envio dos frames dos end devices</p>
<p>No estado atual, o firmware apresenta um pequeno problema no que diz respeito ao gerenciamento dos slots. Isso porque, por exemplo, o frame do end device 3 que deveria ser recebido e exibido no monitor serial do router no slot 3 está sendo exibido no slot 1 ou 2.</p>

