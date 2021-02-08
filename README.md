# DSC 180 Autonomous Systems

Neghena Faizyar, Garrett Gibo, Shiyin Liang

## Data
To get sample data: 
[Link to Data](https://drive.google.com/drive/folders/1wh7EtgtrS8Wi8xBIe1VwzFDBnp751XHv?usp=sharing)

Download this data to put into the data/raw folder.

## Usage 

### Targets
To run any of the following targets, the command is:

```sh
python run.py <target>
```

#### test 

`test` runs our projects test code by extracting, transforming, and then cleaning
the raw GPS tst data such that it could be used. 

#### clean_data

`clean_data` will extract, transform, and clean the raw gps data so
that it can be used for anaylsis.

#### visualize

`visualize` create visualizations for all of our data using bokeh. 
It will plot the line the GPS reports it has traveled and uploads it into 
the vis folder of our repository. 

#### cep

`cep` will create will calculates CEPs, 2DRMS, and then plots and creates a 
graph of the CEP and 2DRMS circles with the datapoints. 


#### robot

`robot` creates an instance of the
[dronekit-sitl](https://dronekit-python.readthedocs.io/en/latest/develop/sitl_setup.html),
which can be used to generate realistic sensor data that can be used
as a template for the following targets.

#### robot_client

`robot_client` provides the interface to connect to a specified robot.
The client connects over tcp or udp and uses the
[MAVLink](https://mavlink.io/en/messages/common.html), standard for
the messages.


## Docker

```sh
# Build
docker build -t <user>/<image-name> -f Docker/Dockerfile .
# Push
docker push <user>/<image-name>
```
