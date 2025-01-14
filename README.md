# Generación de Sistemas de Control para Vehículos Aéreos Usando Algoritmos Evolutivos
Repositorio de proyecto.


## IPD434 - Seminario de Softcomputing
Proyecto de Diego Vásquez W


En este repositorio, se encuentra la base de código de la solución descrita en el paper *Generación de Sistemas de Control para Vehículos Aéreos Usando Algoritmos Evolutivos*.

### Guía de instalación

Para instalar, se encesita un compilador de C++ compatible con C++23, y además CMake.
Se requiere instalar los siguentes paquetes (Debian/Ubuntu):

```bash
sudo apt install build-essential libglu1-mesa-dev libpng-dev
```

Una vez clonado el repostorio, para compilar, usar los siguientes comandos:

```bash
# Crear directorio de output
mkdir build
cd build

# Compilar proyecto
cmake ..
cmake --build . --config Release

# Ejecutar aplicación
./scp
```

Este proyecto está configurado para compilarse con *Unity Build*, para darle posibilidades adicionales de optimización al compilador, pero esto impide la compilación incremental. Para desabilitar esto, se recomienda comentar la siguiente línea en `CMakeLists.txt`:

```cmake
set_target_properties(scp PROPERTIES UNITY_BUILD ON)
```

### Configuración

Dentro del directorio `src`, se encuentra el archivo `config.hpp`:

```cpp
// src/config.hpp
#pragma once

#include "FPType.hpp"
#include "Vec2.hpp"

constexpr unsigned int InputSize = 7;
constexpr unsigned int Hidden1Size = 10;
constexpr unsigned int Hidden2Size = 5;
constexpr unsigned int OutputSize = 2;

constexpr FP PhysicsSimDeltaT = 1.0 / 60.0;
constexpr FP PhysicsSimTargetDroneSpeed = 1.5;

constexpr Vec2 Gravity = {0.0, -10.0};
constexpr FP DroneMass = 10.0;
constexpr FP DroneMomentOfInertia = 5.0;
constexpr FP DroneThrust = 35.0;
constexpr FP DroneTorqueMultiplier = 60.0;
constexpr FP DroneThrustChangeSpeed = 8;

constexpr FP DroneWidth = 0.5;
constexpr FP DroneHeight = 0.25;

constexpr unsigned int GenerationSize = 1000;
constexpr unsigned int SelectNBest = 50;
constexpr unsigned int SimulationsPerDrone = 10;
constexpr unsigned int SimulationThreads = 4;

constexpr FP TrainingDistancePenaltyWeight = 20.0;
constexpr FP TrainingSpeedPenaltyWeight = 20.0;
constexpr FP TrainingAnglePenaltyWeight = 5.0;
constexpr FP TrainingAngularVelPenaltyWeight = 10.0;
constexpr FP TrainingNetworkWeightPenalty = 0.05;

constexpr FP TrainingMaxCoords = 4.0;

constexpr bool TrainingUseRandomInitConditions = false;

constexpr const char* CheckpointFileName = "checkpoint.gen";

// ...
```

Los valores puestos corresponden a los mencionados en el paper, sin embargo, si se desean cambiar, se deja la guía de configuración:

(Los campos marcados con "(*)" **no deben cambiarse**, existe un `static_assert` para evitar cambios inapropiados en la configuración.)


- **(*)** `InputSize`: Cantidad de neuronas en la capa de entrada.
- `Hidden1Size`: Cantidad de neuronas en la primera capa oculta.
- `Hidden2Size`: Cantidad de neuronas en la segunda capa oculta.
- **(*)** `OutputSize`: Cantidad de neuronas en la capa de salida.
- `PhysicsSimDeltaT`: Timestep usado en las simulaciones de entrenamiento.
- `PhysicsSimTargetDroneSpeed`: Velocidad objetivo en las simulaciones de entrenamiento.
- `Gravity`: Vector de aceleración por gravedad.
- `DroneMass`: Masa del dron.
- `DroneMomentOfInertia`: Momento de inercia del dron.
- `DroneThrust`: Fuerza de cada uno de los motores.
- `DroneTorqueMultiplier`: Multiplicador para el factor de torque de la simulación.
- `DroneThrustChangeSpeed`: Velocidad de cambio de respuesta de los motores (más alto, motores responden más rápido).
- `DroneWidth`: Ancho visual del dron (solo gráfico).
- `DroneHeight`: Altura visual del dron (solo gráfico).
- `GenerationSize`: Número de individuos por cada generación.
- `SelectNBest`: Número de drones que quedarán seleccionados para la siguiente generación (y por ende serán usados como base para la próxima generación).
- `SimulationsPerDrone`: Número de simulaciones por cada dron.
- `SimulationThreads`: Número de threads a utilizar para el entrenamiento, recomendable usar la misma cantidad de núcleos de procesamiento de la CPU.
- `TrainingDistancePenaltyWeight`: Peso asociado a la penalización por distancia del objetivo.
- `TrainingSpeedPenaltyWeight`: Peso asociado a la penalización por velocidad del dron.
- `TrainingAnglePenaltyWeight`: Peso asociado a la penalización por diferencia de ángulo con la vertical.
- `TrainingAngularVelPenaltyWeight`: Peso asociado a la penalización por velocidad angular.
- `TrainingNetworkWeightPenalty`: **(NO UTILIZADO)** Peso asociado a la penalización por magnitud de los genes del individuo.
- `TrainingMaxCoords`: Magnitud máxima de cada coordenada al generar el target aleatorio durante las simulaciones de entrenamiento.
- `TrainingUseRandomInitConditions`: Si es `true`, permite que las simulaciones tomen su estado inicial de forma aleatoria.
- `CheckpointFileName`: Nombre del archivo para guardar checkpoints de generación.

### Guía de uso del software.

Aquí se documentan los inputs de teclado e información de uso para cada sección. 

#### Menú principal
Al iniciar el programa, se entra al menú principal. Las teclas [1], [2] y [3] permiten entrar en vuelo manual, vuelo automático y entrenamiento respectivamente.

- La tecla [R] carga un dron aleatorio para la red de control default (la utilizada en la modalidad de vuelo automático).

- La tecla [L] carga el mejor dron de la última generación de entrenamiento.

- La tecla [B] carga el mejor dron que el programa ha visto hasta el momento durante el entrenamiento.

#### Vuelo manual
Usar tecla de [LEFT] y [RIGHT] para encender los motores izquierdo y derecho del dron respectivamente, [R] para reiniciar la simulación.

#### Vuelo automático
Utiliza la red de control cargada, el único control sobre el dron es manipulación del target con el mouse, y la tecla [R] para reiniciar la simulación.

El cuadrado amarillo centrado en el dron representa el rango de targets usados en el entrenamiento, debido a que el dron responde de mejor forma si se sitúa el target dentro del cuadrado.

#### Entrenamiento

En esta sección, se puede guardar y cargar un archivo de checkpoint con [S] y [L], respectivamente (**CUIDADO: al guardar un checkpoint, se sobreescribe cualquier checkpoint anteriormente puesto en el directorio de trabajo.**).

- [P] permite pausar y reanudar el entrenamiento.
- [R] reinicia el entrenamiento y reemplaza la población con una aleatoria.

Al finalizar el entrenamiento de una generación, o al cargar un checkpoint, el mejor dron de la generación queda automáticamente cargado para ser usado en la modalidad de vuelo automático.


#### Controles auxiliares

En las modalidades de vuelo manual y automático, existen controles adicionales para la cámara:

- [V] Cambiar entre cámara fija y centrada en el dron.
- [F1] Centrar cámara en el origen.
- [K] Alejar zoom.
- [L] Acercar zoom.

### Checkpoint precargado

Se provee un checkpoint precargado con 84444 generaciones como referencia:

> [checkpoint.gen (drive.google.com)](https://drive.google.com/file/d/1bQ2BK3MERKck8D7KDFiad8daj2UnoN_Z/view?usp=sharing)

Para utilizarlo, pegarlo en el mismo directorio que `scp` y cargarlo en la modalidad de entrenamiento.