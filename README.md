# pyrobomotra

Python Package / Application to track 2D Robotic Arm Movements.

## Development

### Python3.x

1. Create a Virtual Environment
   
    ```bash
   $ virtualenv -m venv venv
   ```
   
2. Activate Virtual Environment

    ```bash
    $ . venv/bin/activate 
    ```

3. Install the Dependencies

    ```bash
    $ pip install -r requirements.txt
    ```

4. Install `pyrobomotra` as python package for development:

    ```bash
   $ pip install -e .
   ```
   
   This makes the `robot-motion-tracker` binary available as a CLI

### Usage
Run `robot-motion-tracker` binary using command line:

- -c configuration file path/name
- -i Robot ID

```bash
$ robot-motion-tracker -c config.yaml -i <robot-id>
```

### Message Broker (RabbitMQ)

Use the [rabbitmqtt](https://github.com/virtual-origami/rabbitmqtt) stack for the Message Broker

__NOTE__: The `rabbitmqtt` stack needs an external docker network called `iotstack` make sure to create one using `docker network create iotstack`

### Docker

1. To build Docker Images locally use:

    ```bash
    $ docker build -t pyrobomotra:<version> .
    ```

2. To run the Application along with the RabbitMQ Broker connect the container with the `iotstack` network using:

    ```bash
    $ docker run --rm --network=iotstack -t pyrobomotra:<version> -c config.yaml -i <robot-id>
    ```

    __INFO__: Change the broker address in the `config.yaml` file to `rabbitmq` (name of the RabbitMQ Container in _rabbitmqtt_ stack)

3. To run the a custom configuration for the Container use:

    ```bash
    $ docker run --rm -v $(pwd)/config.yaml:/pyrobomotra/config.yaml --network=iotstack -t pyrobomotra:<version> -c config.yaml -i <robot-id>
    ```