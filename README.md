# SEM0590_Boxbot
> Ambiente disponibilizado em repositório da disciplina: https://github.com/EESC-LabRoM/LabRoM_Dinamica_de_Sistemas_Roboticos/tree/main

# ROS 2 Workspace 💻

Este template foi desenvolvido para facilitar a utilização do ROS 2 Humble em ambientes controlados com Docker. Siga os passos abaixo para configurar e rodar o workspace.

---

## Passo 1 - Instalação de Dependências

## 1.2 - Instalando o Docker 🐳

O Docker é a plataforma de contêineres utilizada neste projeto. Recomendamos seguir o guia oficial para instalá-lo no Ubuntu:

[Guia oficial de instalação do Docker no Ubuntu](https://docs.docker.com/engine/install/ubuntu/)

## 1.3 - Configurando o Nvidia Container Toolkit

Importante para computadores com GPU NVIDIA: O Nvidia Container Toolkit é necessário para habilitar o uso de GPUs em contêineres Docker. Siga o guia oficial para instalação:

[Nvidia Container](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).


## Passo 2 - Clone este repositório

<p align="justify">

Para baixar o repositório, execute o comando abaixo:
</p>

```bash
git https://github.com/EESC-LabRoM/LabRoM_Dinamica_de_Sistemas_Roboticos.git
```

Após clonar, copie a pasta `Template` para outro diretório, para executar o template disponibilizado, basta executar os passos abaixo. 

## Passo 3 - Construindo a Imagem Docker

<p align="justify">

Dentro da pasta **Template**. O comando a seguir construirá a imagem docker necessária com imagem noetic com alguns pacotes.
</p>

```bash
docker/scripts/build.sh 
```

Este comando irá preparar o ambiente Docker com todas as dependências necessárias para rodar o ROS 2 Humble, incluindo pacotes e configurações adicionais.

## Passo 4 - Iniciando o Container

<p align="justify">

Para iniciar o container com o ambiente configurado, utilize o script `run.sh`:
</p>

```bash
docker/scripts/run.sh
```

Este comando cria e inicia o container, garantindo que você tenha acesso ao ambiente configurado. O container abrirá o terminal na pasta do workspace, contendo as seguintes subpastas:

- `src`: Local onde você deve colocar os pacotes do ROS 2.
- `build`: Diretório onde os pacotes são construídos.
- `install`: Local onde os pacotes instalados ficam acessíveis.
- `log`: Logs de execução da construção e dos pacotes.

## Passo 5 - Construindo o Workspace

Assim, você estará na pasta que contém os pastas **src**, **build**, **isntall** e **log**. Dentro do container, utilize o comando abaixo para construir o workspace com os pacotes adicionados na pasta `src`:
</p>

```bash
colcon build
```

Esse comando compilará todos os pacotes do workspace. Após a construção, você estará pronto para começar a trabalhar com ROS 2.

## Notas Importantes

1. Certifique-se de que sua GPU NVIDIA está corretamente configurada para o Docker (se aplicável).
2. Caso precise de pacotes adicionais, lembre-se de atualizar o Dockerfile e reconstruir a imagem.
3. Qualquer erro durante a execução deve ser consultado nos logs dentro da pasta `log`.
