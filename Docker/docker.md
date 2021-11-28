# Docker

## Install (Ubuntu)
https://docs.docker.com/engine/install/ubuntu/

1. Update the apt package index and install packages to allow apt to use a repository over HTTPS
```
$ sudo apt-get update

$ sudo apt-get install \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg-agent \
    software-properties-common
```

2. Add Docker’s official GPG key
```
$ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add
```

3. Use the following command to set up the stable repository
```
$ sudo add-apt-repository \
   "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
   $(lsb_release -cs) \
   stable"
```

4. install the latest version of Docker Engine and containerd
```
sudo apt-get install docker-ce docker-ce-cli containerd.io
```


## Unistall

1. Uninstall the Docker Engine, CLI, and Containerd packages:
```
$ sudo apt-get purge docker-ce docker-ce-cli containerd.io
```

2. Images, containers, volumes, or customized configuration files on your host are not automatically removed. To delete all images, containers, and volumes:
```
$ sudo rm -rf /var/lib/docker
```

3. You must delete any edited configuration files manually




---

## Concepts

* Dockerfile: bleprint to build a docker image
* Image: template to run docker containers
* Container: running preocess


### Directory Tree

```
project_name
├─ src
│   └─ ...
├─ Dockerfile
├─ .dockerignore
└─ docker-compose.yml
```


## Workflow

1. Pull a docker image
   ```
   docker pull [image_name]:[image_version]
   ```
2. Creating a dockerfile
3. Build the dockerfile
    ```
    docker build -t [image_name]:[image_version]
    ```
4. Push the dockerfile
   ```
   docker push
   ```
5. Run a container
   ```
   docker run
   ```


------



## Docker compose
Docker compose is a tool for defining and running multi-container Docker applications



