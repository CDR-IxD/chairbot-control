const WebSocket = require('ws');

const ws = new WebSocket('ws://ubuntu-cdr.local:3000');

ws.on('open', function open() {
    ws.send('something');
})
