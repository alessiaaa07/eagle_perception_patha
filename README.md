# Rilevamento dei coni dal LiDAR

Questo progetto implementa un nodo ROS 2 in C++ per il rilevamento dei coni
a partire dalle nuvole di punti del LiDAR.

Il nodo rimuove il suolo, raggruppa i punti rimanenti, filtra i cluster
in base a dimensione e forma, e pubblica le posizioni dei coni nel frame
`base_link`.

## Requisiti

- Docker
- Docker Compose
- Git

## Scaricare il bag

Scaricare il file ZIP dal link fornito nel task ed estrarlo nella cartella:

```text
data/
```

La cartella finale deve contenere:

```text
data/perception_bag/
```

## Costruire l’immagine

Dalla cartella principale del repository:

```bash
docker compose build
```

## Avviare il container

```bash
docker compose up -d
```

Entrare nel container:

```bash
docker compose exec eagle bash
```

## Compilare la workspace

Dentro il container:

```bash
cd ~/ws
colcon build
source install/setup.bash
```

## Avviare il nodo

In un terminale del container:

```bash
cd ~/ws
source install/setup.bash
ros2 launch cone_detection cone_detection.launch.py
```

## Riprodurre il bag

In un secondo terminale del container:

```bash
cd ~/data
ros2 bag play perception_bag
```

## Visualizzare i risultati

In un terzo terminale del container:

```bash
rviz2
```

In RViz impostare il **Fixed Frame** su `base_link`, quindi aggiungere:

- il point cloud del LiDAR;
- il topic `/perception/cones_markers`;
- il topic `/perception/cones`, se si vuole leggere il messaggio.