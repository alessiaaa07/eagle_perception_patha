# Report — Rilevamento dei coni dal LiDAR

## Approccio

Questo progetto implementa un nodo ROS 2 scritto in C++ che rileva i coni
a partire dalle nuvole di punti prodotte da un LiDAR.

Il nodo si iscrive al topic `/lidar/points`, riceve una scansione LiDAR alla
volta e la elabora per individuare i coni presenti intorno al veicolo.

Le posizioni dei coni vengono pubblicate nel frame del veicolo, `base_link`.

## Trasformazione dei punti

I punti del LiDAR sono espressi nel frame del sensore, chiamato `lidar`.
Tuttavia, il risultato richiesto deve essere nel frame del veicolo,
`base_link`.

Per questo motivo il nodo utilizza la trasformazione statica pubblicata nel
bag tramite `/tf_static`.

Per ogni scansione, il nodo recupera la trasformazione da `lidar` a
`base_link` e applica la matrice di trasformazione a ciascun punto.

In questo modo, tutte le elaborazioni successive vengono effettuate direttamente
nel frame del veicolo.

## Rimozione del suolo

Dopo la trasformazione nel frame `base_link`, il nodo analizza l’altezza
dei punti lungo l’asse `z`.

Vengono mantenuti solo i punti la cui altezza è compresa tra i parametri
`ground_z_min` e `ground_z_max`.

I punti troppo vicini al suolo vengono scartati, perché appartengono
probabilmente al terreno. Vengono scartati anche i punti troppo alti,
che non possono appartenere a un cono.

I punti rimanenti vengono inseriti in una nuvola di punti PCL,
che viene utilizzata per la fase successiva di clustering.

## Clustering

I punti non appartenenti al suolo vengono raggruppati tramite l’algoritmo
di clustering euclideo fornito dalla libreria PCL.

Il nodo utilizza un albero KD per cercare i punti vicini e raggruppa insieme
i punti la cui distanza è inferiore al parametro `cluster_tolerance`.

Per ogni gruppo vengono applicati dei limiti di dimensione:

- numero minimo di punti;
- numero massimo di punti.

In questo modo si evita di considerare rumore isolato o oggetti molto grandi,
come edifici, barriere o altre strutture presenti attorno alla pista.

## Riconoscimento dei coni

Non tutti i cluster individuati corrispondono a un cono.

Per ogni cluster il nodo calcola:

- larghezza, cioè la differenza tra il valore massimo e minimo di `x`;
- profondità, cioè la differenza tra il valore massimo e minimo di `y`;
- altezza, cioè la differenza tra il valore massimo e minimo di `z`.

Un cluster viene considerato un cono solo se le sue dimensioni rientrano
nei limiti definiti nei parametri:

- `max_cone_width`
- `max_cone_depth`
- `min_cone_height`
- `max_cone_height`

Questi parametri permettono di scartare oggetti troppo larghi, troppo alti
o troppo piccoli per essere coni.

## Stima della posizione

Per ogni cluster riconosciuto come cono, il nodo calcola la media delle
coordinate `x` e `y` dei punti che lo compongono.

La coordinata `z` viene impostata a `0`, perché la posizione richiesta è
il centro del cono sul suolo.

La posizione viene quindi pubblicata nel frame `base_link`.

## Pubblicazione dei risultati

Il nodo pubblica tre tipi di messaggi:

| Topic | Messaggio | Contenuto |
|---|---|---|
| `/perception/cones` | `eagle_perception_interfaces/msg/ConeArray` | Posizioni dei coni rilevati |
| `/perception/cones_markers` | `visualization_msgs/msg/MarkerArray` | Marker per visualizzare i coni in RViz |
| `/perception/processing_time` | `eagle_perception_interfaces/msg/ProcessingTime` | Tempo impiegato per elaborare la scansione |

Tutti i messaggi pubblicati utilizzano il timestamp della scansione LiDAR
da cui sono stati generati, come richiesto dal task.

## Misurazione del tempo

Il tempo di elaborazione viene misurato per ogni scansione.

Prima di iniziare l’elaborazione viene salvato il tempo corrente.
Al termine delle operazioni viene calcolata la differenza, espressa in
millisecondi.

Il risultato viene pubblicato nel messaggio `ProcessingTime` sul topic:

```text
/perception/processing_time
```

Il budget disponibile è di 100 ms per scansione, poiché il LiDAR produce
una scansione ogni 100 ms.

## Problemi incontrati

Durante lo sviluppo è stato necessario capire come utilizzare correttamente
la trasformazione tra il frame del LiDAR e quello del veicolo.

Inizialmente i punti venivano elaborati direttamente nel frame del sensore.
Tuttavia, il LiDAR non è montato esattamente  nell’origine del veicolo e non
è perfettamente livellato.

Per risolvere questo problema, il nodo utilizza `/tf_static` per ottenere la
trasformazione da `lidar` a `base_link` prima di effettuare la rimozione
del suolo e il clustering.

Un altro aspetto importante è stato distinguere i coni dagli altri oggetti
presenti nella scena. Per questo motivo, oltre alla rimozione del suolo,
sono stati applicati dei filtri basati su dimensione e altezza dei cluster.

## Possibili miglioramenti

Alcuni miglioramenti possibili sono:

- utilizzare l’intensità dei punti per distinguere i coni blu, gialli e arancioni;
- migliorare la stima della posizione del cono sul suolo;
- ottimizzare il tempo di elaborazione per ridurre i casi che superano 100 ms;
- aggiungere un filtro temporale per ridurre falsi positivi tra scansioni consecutive;
- rendere i parametri modificabili durante l’esecuzione del nodo.

## Risultati
Il nodo è stato testato riproducendo il bag `perception_bag`.

Durante l’esecuzione, il nodo riceve le scansioni LiDAR, rimuove il suolo,
raggruppa i punti rimanenti e pubblica le posizioni dei coni rilevati.

I coni possono essere visualizzati in RViz tramite il topic:

```text
/perception/cones_markers
```