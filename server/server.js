/**
 * Servidor WebSocket para Rede LoRa Mesh
 * Executar com: node server.js
 */

const WebSocket = require('ws');
const PORT = 8080;

// Inicializa o servidor WebSocket na porta 8080
const wss = new WebSocket.Server({ port: PORT });

// Conjunto de clientes web conectados (para broadcasting de logs e topologia)
const webClients = new Set();

// Tabela de Roteamento em memória: mapeia o ID do Nó (String) para a instância WebSocket do Router
const routingTable = new Map();

// Armazena a topologia atual para enviar aos novos clientes web: { "routerId": [node1, node2] }
const networkTopology = {};

// Função auxiliar para enviar a topologia de rede para todas as interfaces web
function broadcastTopology() {
    const msg = JSON.stringify({ type: 'topology', data: networkTopology });
    webClients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) {
            client.send(msg);
        }
    });
}

// Função auxiliar para reencaminhar as mensagens de dados/logs para as interfaces web
function broadcastToWeb(messageObj) {
    const msg = JSON.stringify(messageObj);
    webClients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) {
            client.send(msg);
        }
    });
}

wss.on('connection', (ws, req) => {
    console.log(`[+] Nova ligação detetada: ${req.socket.remoteAddress}`);

    ws.on('message', (message) => {
        try {
            const data = JSON.parse(message);
            console.log('[>] Mensagem recebida:', data);

            // 1. Identificação da Interface Web
            if (data.type === 'login_web') {
                ws.clientType = 'web';
                webClients.add(ws);
                console.log('    -> Interface Web registada.');
                // Enviar topologia atual logo após a ligação
                ws.send(JSON.stringify({ type: 'topology', data: networkTopology }));
            } 
            
            // 2. Identificação do Router
            else if (data.type === 'login_router') {
                ws.clientType = 'router';
                ws.routerId = String(data.src);
                
                // Registar o próprio router na tabela
                routingTable.set(ws.routerId, ws);
                networkTopology[ws.routerId] = data.nodes || [];
                
                // Registar cada nó do router na tabela de roteamento
                if (Array.isArray(data.nodes)) {
                    data.nodes.forEach(node => {
                        routingTable.set(String(node), ws);
                    });
                }
                console.log(`    -> Router [${ws.routerId}] registado com os nós: ${data.nodes}`);
                
                // Atualizar a interface web
                broadcastTopology();
            } 
            
            // 3. Tráfego da Rede (Comandos ou Dados)
            else {
                // Se a mensagem vem da Web (Comando) -> Encaminhar para o Router específico
                if (ws.clientType === 'web') {
                    const targetNode = String(data.dst);
                    const targetWs = routingTable.get(targetNode);
                    
                    if (targetWs && targetWs.readyState === WebSocket.OPEN) {
                        targetWs.send(JSON.stringify(data));
                        console.log(`    -> Comando reencaminhado para o Nó [${targetNode}] através do Router associado.`);
                        // Opcional: fazer o eco da mensagem de volta para a Web para atualizar o Log de quem enviou
                        broadcastToWeb(data);
                    } else {
                        console.log(`    -> Erro: O destino [${targetNode}] não está registado na Tabela de Roteamento.`);
                    }
                } 
                // Se a mensagem vem do Router (Dados) -> Fazer broadcast para a(s) interface(s) Web
                else if (ws.clientType === 'router') {
                    console.log(`    -> Dados do Nó [${data.src}] reencaminhados para a Interface Web.`);
                    broadcastToWeb(data);
                }
            }
        } catch (err) {
            console.error('Erro ao processar mensagem JSON:', err.message);
        }
    });

    ws.on('close', () => {
        console.log('[-] Cliente desconectado.');
        
        if (ws.clientType === 'web') {
            webClients.delete(ws);
        } else if (ws.clientType === 'router') {
            // Remover router e os seus nós da tabela de roteamento
            if (ws.routerId) {
                const nodes = networkTopology[ws.routerId] || [];
                nodes.forEach(node => routingTable.delete(String(node)));
                routingTable.delete(ws.routerId);
                delete networkTopology[ws.routerId];
                
                console.log(`    -> Router [${ws.routerId}] removido da Tabela de Roteamento.`);
                broadcastTopology(); // Atualiza a web informando que o router caiu
            }
        }
    });
});

console.log(`=== Servidor LoRa Mesh iniciado na porta ${PORT} ===`);