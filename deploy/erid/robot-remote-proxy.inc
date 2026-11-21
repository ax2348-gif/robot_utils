# Pass-through to the Pi (included by robot-remote.conf).
proxy_pass http://robot_pi;
proxy_http_version 1.1;
proxy_set_header Connection "";
proxy_set_header Host $host;
proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;   # the client goes last: the robot reads that one
proxy_set_header X-Forwarded-Proto $scheme;
# streams (/stream, /events) never end and must not be held back
proxy_buffering off;
proxy_request_buffering off;
proxy_cache off;
proxy_read_timeout 1d;
proxy_send_timeout 1d;
proxy_connect_timeout 5s;
