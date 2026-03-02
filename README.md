# LoRaMesh Protocol - Comunicação Descentralizada para Longo Alcance

![GitHub license](https://img.shields.io/badge/license-MIT-blue.svg)
![Platform](https://img.shields.io/badge/Platform-ESP32-red.svg)
![Field](https://img.shields.io/badge/Field-Control%20%26%20Automation-green.svg)

## 📖 Sobre o Projeto
Este repositório contém o desenvolvimento de um protocolo de rede **Mesh** baseado na tecnologia **LoRa**. O objetivo é permitir a criação de redes descentralizadas, onde cada nó atua como roteador para estender o alcance da comunicação em cenários de automação industrial ou monitoramento remoto.

O projeto foi desenvolvido focando na robustez da camada de enlace e na eficiência do roteamento multi-hop, para garantir a integridade dos dados em ambientes ruidosos.

## 🛠️ Especificações Técnicas
* **Hardware Base:** ESP32 + Rádio LoRa (SX127x)
* **Frequência:** 915 MHz (Configurável para 433 MHz)
* **Camada Física:** LoRa Modulation
* **Topologia:** Mesh Dinâmica (Ad-Hoc)

## 🚀 Funcionalidades Implementadas
- [x] **Encaminhamento Multi-hop:** Saltos inteligentes entre nós para alcance estendido.
- [x] **Auto-Healing:** A rede se reconfigura automaticamente se um nó cair.
- [x] **Controle de Colisão (CSMA):** Verificação de canal ocupado antes da transmissão.
- [x] **Baixo Consumo:** Gerenciamento de energia para operação em baterias.

## 📦 Estrutura do Pacote (Header)
O protocolo utiliza um cabeçalho customizado para gerenciar o roteamento:

| Byte | Nome | Descrição |
| :--- | :--- | :--- |
| 0x00 | `SrcID` | ID do nó de origem (Original) |
| 0x01 | `DestID` | ID do nó de destino final |
| 0x02 | `PrevID` | ID do último nó que retransmitiu |
| 0x03 | `TTL` | Time-to-Live (Máximo de saltos) |
| 0x04 | `Type` | Tipo de pacote (DATA, ACK, HELLO) |

## 🔧 Configuração e Uso
1. **Requisitos:**
   - [PlatformIO](https://platformio.org/) ou Arduino IDE.
   - Bibliotecas: `LoRa.h` (Sandeep Mistry) ou `RadioLib`.

2. **Instalação:**
   ```bash
   [git clone [https://github.com/seu-usuario/loramesh-protocol.git](https://github.com/seu-usuario/loramesh-protocol.git)](https://github.com/GabrielSperoto/Projeto_loramesh.git)
