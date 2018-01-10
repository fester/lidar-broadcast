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
