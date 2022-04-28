FROM ubuntu:20.04

RUN apt-get update && apt-get install -y curl git docker-compose jq

COPY compose/submit/start.sh /start.sh
RUN chmod +x /start.sh

CMD ["/start.sh"]

