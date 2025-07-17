# Botón Físico de Pánico
Códigos para la tesis: “Diseño e implementación de una red de comunicación de emergencia para el campus Balzay”

Este repositorio contiene los códigos fuente utilizados para la implementación de botones físicos de pánico basados en tecnología LoRaWAN. El objetivo es facilitar la comunicación de alertas de emergencia dentro del campus Balzay de la Universidad de Cuenca.

Dispositivos Compatibles
Los códigos han sido desarrollados y validados en la placa BSFrance LoRa32u4 II versión 1.3, que integra un microcontrolador ATmega32u4 y un módulo Semtech SX1276. También pueden adaptarse a otros dispositivos que utilicen el mismo microcontrolador, siempre que se verifique y ajuste el mapeo de pines necesario para el control del módulo LoRa.

Redireccionador de Paquetes (Forwarder)
El forwarder o redireccionador es un componente opcional que permite enviar los mensajes LoRaWAN a través de una API personalizada, desarrollada por el Departamento de Tecnologías de la Información y Comunicación (DTIC) de la Universidad de Cuenca.

⚠️ Nota: El uso del redireccionador solo es necesario si se desea integrar la red de botones físicos con servicios externos a través de una API. No es obligatorio para el funcionamiento básico de los dispositivos con TTN o servidores LoRaWAN estándar.

Archivos de Configuración
El archivo config contiene parámetros necesarios para la operación del redireccionador. No incluye información sensible del proyecto, ya que estos datos deben ser personalizados por cada usuario para sus propios entornos y credenciales.

Seguridad y Personalización
🔒 Importante: Todos los archivos deben ser revisados y modificados por el usuario para incorporar las credenciales y claves adecuadas según su propia infraestructura y proyecto. Esto incluye claves de acceso LoRa, tokens de autenticación, direcciones de servidores, entre otros.

# Physical Pannic Button
Code for the Thesis: "Design and Implementation of an Emergency Communication Network for the Balzay Campus", the documnet was written in spanish though the codes are available in English.

This repository contains the source code used for the development of physical panic buttons based on LoRaWAN technology. The goal is to enable reliable emergency alert communication within the Balzay campus of the University of Cuenca.

Compatible Devices
The code has been tested and validated on the BSFrance LoRa32u4 II version 1.3 board, which integrates an ATmega32u4 microcontroller and a Semtech SX1276 LoRa module.
It can also be adapted for use with other ATmega32u4-based devices, as long as the pin mapping is properly configured to control the LoRa module.

Packet Forwarder
The forwarder is an optional component that allows LoRaWAN messages to be sent through a custom API developed by the Department of Information and Communication Technologies (DTIC) at the University of Cuenca.

⚠️ Note: The forwarder is only required if you intend to integrate the panic button network with external services through a custom API. It is not necessary for basic operation with TTN or standard LoRaWAN servers.

Configuration Files
The config file contains the necessary parameters to operate the forwarder. It does not include any sensitive information, and must be personalized by each user according to their own system.

Security and Customization
🔒 Important: All files must be reviewed and edited by users to include their own project credentials and security keys. This includes LoRaWAN keys, authentication tokens, server addresses, and other project-specific parameters.

