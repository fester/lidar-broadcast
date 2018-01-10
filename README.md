# Usage instructions

Minimal python version is 3.5

Install dependencies and config:

```
pip install -r requirements.txt
cp config.yaml.example config.yaml
```

## Server

Edit config.yaml and specify:
* tty where the device is attached
* Optionally - add host to specify interface to bind on

Run server:
```
python server.py
```


## Client

Edit config.yaml and specify:
* host to specify remote host to connect to
* port to specify remote port 

Run simple client
```
python sub.py
```

## Data packages

The server broadcasts Sweep scans through a 0MQ pub socket. Messages are in JSON, with the following schema:

```json
{
   "type":"object",
   "properties":{
      "time":{
         "description":"Timestamp of a scan in seconds",
         "type":"float"
      },
      "scan":{
         "description":"List of measurements in the scan",
         "type":"array",
         "items":{
            "type":"object",
            "properties":{
               "distance":{
                  "type":"float",
                  "description":"Distance to the point, meters"
               },
               "angle":{
                  "type":"float",
                  "description":"Angle to the point, degrees"
               },
               "strength":{
                  "type":"integer",
                  "description":"Signal strength for the point, 0..255"
               }
            }
         }
      }
   }
}
```
