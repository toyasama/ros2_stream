#!/bin/bash

echo "[+] Autorisation X11 pour Docker..."
xhost +local:docker

echo "[+] Démarrage du conteneur..."
docker compose up --build

echo "[+] Révocation de l'autorisation X11..."
xhost -local:docker
