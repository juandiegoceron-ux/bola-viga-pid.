# Sistema de Control Bola-Viga (Ball and Beam) - PID en Arduino

Este repositorio contiene la implementación física y de software de un sistema de control de lazo cerrado **Bola-Viga (Ball and Beam)**, diseñado e implementado por estudiantes de **Ingeniería Biomédica** para la asignatura de **Sistemas Dinámicos (2026)** en la **Universidad Autónoma de Occidente (UAO)**.

El objetivo del sistema es estabilizar una esfera en el centro geométrico de una viga basculante de $38\text{ cm}$ (Setpoint = $19\text{ cm}$) controlando el ángulo de inclinación mediante un servomotor y sensando la posición en tiempo real con un sensor ultrasónico.

## 🚀 Características del Controlador
El controlador implementado es un **PID digital discreto** diseñado heurísticamente y optimizado para robustecer el comportamiento físico real ante ruidos de lectura y limitaciones mecánicas:
* **Filtro de Mediana Móvil (N = 5):** Filtro no lineal integrado para remover spikes y picos de ruido acústico típicos del sensor ultrasónico HC-SR04.
* **Derivada sobre la Medición:** Algoritmo estructurado calculando la acción derivativa ($D$) sobre la posición medida de la bola en lugar del error, eliminando por completo el fenómeno de *Derivative Kick* durante cambios bruscos del sistema.
* **Anti-Windup Condicional:** Algoritmo que congela la acumulación de la acción integral ($I$) cuando el actuador alcanza sus límites de saturación física ($\pm 20^\circ$).
* **Resolución por Microsegundos:** Uso de control PWM micrométrico en el servomotor (`writeMicroseconds`) en lugar de control clásico por grados enteros para mayor suavidad.
* **Memoria de Estado:** Retención inteligente de la última lectura útil ante pérdidas temporales del eco del sensor ultrasónico.

## 🛠️ Componentes y Conexiones
El circuito eléctrico consta de los siguientes componentes conectados a una tarjeta **Arduino UNO**:

| Componente | Pin Arduino | Tipo de Señal | Función |
|:---|:---:|:---:|:---|
| **HC-SR04 (Trig)** | Pin 7 | Salida Digital | Disparo de pulso ultrasónico |
| **HC-SR04 (Echo)** | Pin 8 | Entrada Digital | Medición del tiempo de eco |
| **Servo SG-5010 / S3003** | Pin 9 | Salida PWM ($\mu$s) | Actuador mecánico de inclinación |

*Nota: Para evitar caídas de tensión, el servomotor se alimenta mediante una fuente externa de 5V compartiendo la tierra (GND) común con el Arduino.*

## 📂 Estructura del Proyecto
* `/src`: Código fuente definitivo de Arduino (`.ino`).
* `LÉAME.md`: Documentación del proyecto.

## 👥 Autores
* **Juan Diego Cerón Duque**
* **Juan Sebastián Carrillo Eraso**
* **Miguel Ángel Moreano Cabrera**

*Ingeniería Biomédica - Universidad Autónoma de Occidente (UAO)*
