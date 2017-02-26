/* Autor : Santiago Ventura Gomez . 2017
Curso Experto Universitario en Robótica , Programación e Impresión 3D
Asignatura : App Inventor
Profesor : Jorge Campo
Actividad 4 : Solicitud de informacion a la placa
*/

/*
Objetivo : 

Trabajo: Solicita información a la placa

La actividad consiste en diseñar una aplicación que proporcione información sobre un sensor o sensores a la placa ZUM. 
Dicha petición se hará por voz, mientras que el resultado será devuelto también por voz. 
(Ejemplo: Al utilizar la palabra "luz" el móvil contesta el valor del sensor de luz de forma hablada).

 La aplicación estará diseñada con algún objetivo o función concreta a elegir (Ejemplo: estación meteorológica). 
 El aspecto de la aplicación tiene que ser acorde a la función elegida. Se valorará positivamente el utilizar diferentes sensores.
Se debe construir un robot que recorra un circuito delimitado por paredes

Acerca de mi solución a la actividad:

He diseñado un robot móvil que avanza en linea recta por un suelo con franjas alternativas Negro-Blanco de 5 cms de ancho ,
hacia una pared frontal situada aproximadamente a 100 cms de distancia de su origen .
Nos valemos de un potenciómetro como variador de su velocidad de avance antes de comenzar , y una vez en movimiento ,
de dos formas de medir su posición y velocidad : Sensores IR y sensor de ultrasonidos
Por una parte , los sensores IR iran midiendo los cambios de color de las franjas del suelo para medir el avance
Por otro lado , simultaneamente , el sensor US , irá midiendo la distancia a la pared frontal en cada momento.

La placa se comunica por bluetooth con la aplicacion del movil " Mido_mientras_Hablo.aia " de la que recibe
instrucciones por voz para Avanzar , Parar , así como para informar de los valores del Potenciómetro , 
de los sensores IR , de la distancia a la pared y de la velocidad de avance del robot calculada con la informacion
recibida por el sensor de ultrasonidos y por los IR.
*/

/* Para definir los lados Izquierda y Derecha del Robot se hace visto desde la parte trasera y mirando en la dirección
    de avance del mismo */
    
/* Elementos de construcción del Robot :

    * Para las piezas del Chasis , me he descargado los archivos STL del Printbot Evolution , y he utilizado la base de 3 mm
      Todas las piezas están impresas en una impresora Hephestos 2 
    * Movimiento de las Ruedas : Robot diferencial de tres puntos de apoyo , que cuenta con dos servomotores de movimiento continuo para las ruedas
      y un apoyo fijo trasero
    * Sensor de ultrasonidos dirigido al frente , fijo , que mide la distancia desde el robot a la pared frontal
    * Sensores IR a ambos lados del robot . Ambos dos miden los cambios de color de las franjas del suelo rallado en blanco y negro.
      con franjas de 5 cms de ancho . Uno de ellos sensible al cambio de blanco a negro , mientras que el otro sensible al cambio de negro a blanco
    * Boton pulsador que permite de forma manual parar o poner en marcha el robot
    * Potenciómetro que regula la velocidad de avance de ambas ruedas
    * Comunicación abierta en todo momento por el canal de bluetooth de la placa ZUM con el móvil que tiene abierta la aplicación
    * Servomotor que controla la orientacion del sensor US ( No es necesario pero aprovecho la construccion general del robot que tengo )
 */
 
 /* Asignación de pines :
 
   * Sensor IRizqda       :  Pin 2
   * Sensor IRdrcha       :  Pin 3
   * Servo RuedaDrcha     :  Pin 4
   * Servo RuedaIzqda     :  Pin 5
   * Servo ControlUS      :  Pin 7
   * PulsadorInicio       :  Pin 8
   * Sensor ultrasonidos  : TRIG = PIN 11 , ECHO = PIN 12
   * Potenciómetro        :  Pin A0

*/
/*  Valor de velocidad :

   * Avance Robot :
       Rueda Derecha : Va_drcha
       Rueda Izquierda : Va_izqda
   
   * Parar :
       Rueda Derecha : 90º
       Rueda Izquierda : 90º
  */
  /* Estados de la máquina :
  
  * S0  : Motores apagados , se regula el valor del potenciometro , abierto el puerto de comunicacion
  * S1  : Todos los elementos del robot funcionando salvo el potenciometro
 */
 
 /* Eventos de la máquina :
 
   * E1 : Pulsacion del boton
   * E2 : Distancia a la pared < d_limite
   
 */
 

/********************************* DEFINICION DE LIBRERIAS Y VARIABLES GLOBALES **********************************************/

#include <Servo.h>                            // Libreria Arduino que controla funciones especiales de los servomotores
#include <BitbloqUS.h>                        // Libreria Bitbloq que sirve `para controlar un sensor de ultrasonidos
#include <SoftwareSerial.h>                   // Libreria Arduino de comunicaciones serie
#include <BitbloqSoftwareSerial.h>            // Libreria Bitbloq de comunicaciones serie

Servo RuedaIzqda;                             // Declaracion de variable tipo Servo para el motor que mueve la rueda Izquierda
Servo RuedaDrcha;                             // Declaracion de variable tipo Servo para el motor que mueve la rueda Derecha
Servo ControlUS;                              // Declaracion de variable tipo Servo para el microservo que controla la orientacion del sensor US

bqSoftwareSerial bluetooth(0,1,19200);        // Declaracion de puerto de comunicaciones por bluetooth a 19200 baudios de velocidad
US ultrasonidos(11,12);                       // Declaracion de variable tipo US para el sensor de ultrasonidos y pines TRIG y ECHO donde
                                              // está conectado
int IRizqda = 2;                              // Declaracion de pin asignado al sensor IR de la izquierda
int IRdrcha = 3;                              // Declaracion de pin asignado al sensor IR de la derecha
int Pulsador = 8;                             // Declaracion del pin asignado al boton que iniciará el programa
int Acelerador = A0;                          // Declaracion del pin signado al potenciometro que regula la velocidad de las ruedas

float d_limite = 10;                           // Distancia minima limite en cms desde el sensor a la pared frontal para hace que el robot
                                              // pare automaticamente para no chocar
float distancia_inic;                         // Variable que guarda el valor de distancia en cm recibido desde el sensor US al inicio                                              
float distancia;                              // Variable que guarda el valor de distancia en cm recibido desde el sensor US
float distancia_recorrida;                    // Variable que guarda el valor de la distancia recorrida por el robot
int Va_derecha = 90;                          // Velocidad de avance de la rueda derecha , en grados
int Va_izquierda = 90;                        // Velocidad de avance de la rueda izquierda , en grados
int Aceleracion = 0;                          // Variable que guarda la aceleracion dependiendo del valor que envie el potenciometro
int estado = 0;                               // Variable que guarda el estado de mi maquina
volatile int contador = 0;                    // Variable que guarda el numero de segmentos de 5 cm que ha avanzado el robot

String mensaje = "";                          // Variable encargada de almacenar la cadena de caracteres recibida por el canal de bluetooth

float T_inic = 0;                             // Valor inicial de tiempo al comenzar a moverse el robot                                             
float T_final = 0;                            // Valor del instante en que el robot se para
float T_empleado = 0;                         // Valor del tiempo que el robot ha estado en movimiento
float velocidad = 0;                          // Valor de la velocidad del robot en cm/segundo


/***********************    DEFINICION DE FUNCIONES LIGADAS Al MOVIMIENTO DE LAS RUEDAS DEL ROBOT    **********************************/


void avanzar(Servo Rueda_D,Servo Rueda_I,int Va_D,int Va_I){                 // Funcion que hace avanzar al robot
        Rueda_D.write(Va_D);
        Rueda_I.write(Va_I);        
}

void parar(Servo Rueda_D,Servo Rueda_I){                                     // Funcion que hace pararse al robot
        Rueda_D.write(90);
        Rueda_I.write(90);        
}

/**********************    DEFINICION DE LA FUNCION LIGADA A LAS INTERRUPCIONES DE LOS SENSORES IR    **********************/

void contar_rallas(){                         // Cada vez que se provoca una interrupcion por parte de un sensor IR
                                              // se incrementa el contador en una unidad y se vuelve al programa ppal
        contador ++;
}

void setup() {

  
  RuedaDrcha.attach(4);                       // Declaracion del pin al que está conectado el servomotor de la rueda derecha
  RuedaIzqda.attach(5);                       // Declaracion del pin al que está conectado el servomotor de la rueda izquierda
  ControlUS.attach(7);                        // Declaracion del pin al que está conectado el microservo del sensor US
    
  pinMode(IRizqda,INPUT);                     // Declaramos el pin del sensor IR izquierda como entrada
  pinMode(IRdrcha,INPUT);                     // Declaramos el pin del sensor IR derecha como entrada
  pinMode(Pulsador,INPUT);                    // Declaramos el pin del pulsador como entrada
  pinMode(Acelerador,INPUT);                  // Declaramos el pin del potenciometro como entrada
    
  attachInterrupt(0,contar_rallas,FALLING);   // Interrupcion provocada por el sensor IR izquierdo al pasar de 1 a 0
  attachInterrupt(1,contar_rallas,RISING);    // Interrupcion provocada por el sensor IR derecho  al pasar de 0 a 1
  
  ControlUS.write(90);                        // Orientamos el sensor US al frente
  parar(RuedaDrcha,RuedaIzqda);               // Inicialmente las dos ruedas paradas
  distancia_inic = ultrasonidos.read();       // Medimos la distancia inicial a la pared con el US y se lo asignamos a distancia_inic
  distancia = distancia_inic;                 // En un principio igualamos la distancia a la medida en el primer instante
}



void loop() {

/****************************************                  INICIO PROGRAMA PRINCIPAL              ********************************/

/******************************************    RECEPCION DE MENSAJES DESDE EL MOVIL   *******************************************/

if (bluetooth.available() >0){
      mensaje = bluetooth.readString(); 
}

/******************************************   DESCRIPCION DE LA LOGICA DE LA MAQUINA  ********************************************/

if (estado == 0 && digitalRead(Pulsador) == 1){       // Si el robot está parado y pulsamos boton ...
      while (digitalRead(Pulsador) == 1){             // Rutina para evitar efecto rebote
          delay(10);
      }
      estado = 1;                                     // Cambiamos a estado 1
      T_inic = millis();                              // Tomamos nota del instante de reloj
      distancia_inic = ultrasonidos.read();           // Leemos distancia en el US
      contador = 0;                                   // Ponemos a cero el contador de rallas detectadas
      attachInterrupt(0,contar_rallas,FALLING);       // Interrupcion provocada por el sensor IR izquierdo al pasar de 1 a 0
      attachInterrupt(1,contar_rallas,RISING);        // Interrupcion provocada por el sensor IR derecho  al pasar de 0 a 1
}
else if (estado == 1 && digitalRead(Pulsador) == 1){  // Si el robot está en marcha y pulsamos boton ...
      while (digitalRead(Pulsador) == 1){             // Rutina para evitar efecto rebote
          delay(10);
      }
      estado = 0;                                     // Cambiamos a estado 0
      detachInterrupt(0);                             // Desconectamos la interrupcion definida en el pin 2
      detachInterrupt(1);                             // Desconectamos la interrupcion definida en el pin 3
}
else if (estado == 1 && distancia < d_limite){        // Si el robot está en marcha y nos acercamos demasiado al obstaculo ...
      estado = 0;                                     // Cambiamos a estado 0
      detachInterrupt(0);                             // Desconectamos la interrupcion definida en el pin 2
      detachInterrupt(1);                             // Desconectamos la interrupcion definida en el pin 3
}


/*************************************    DESCRIPCION DE LA ACCIONES DE LA MAQUINA   ************************************************/

switch (estado){                                                              // Revisamos estado del robot y actuamos

  case 0:                                                                     // Si estado = 0 entonces ....
          parar(RuedaDrcha,RuedaIzqda);                                       // Paramos motores
          Aceleracion = map(analogRead(Acelerador),0,1023,0,90);              // Revisamos el valor del potenciometro y le asignamos
                                                                              // un valor proporcional en grados a la velocidad de giro
                                                                              // de las ruedas
          Va_derecha = 90 + Aceleracion;                                      // Asignamos el valor en grados con el que tiene que avanzar la rueda derecha 
                                                                              // en funcion del valor del potenciometro ( de 90º a 180 º )
          Va_izquierda = 90 - Aceleracion;                                    // Asignamos el valor en grados con el que tiene que avanzar la rueda izquierda
                                                                              // en funcion del valor del potenciometro ( de 90º a 0º )
          distancia_recorrida = distancia_inic - distancia;                   // Calculamos distancia recorrida por el robot
          T_empleado = T_final - T_inic;                                      // Calculamos el tiempo transcurrido en movimiento por el robot
          if (T_empleado != 0){
                  velocidad = 1000*distancia_recorrida/T_empleado;            // Calculamos la velocidad empleada por el robot en su movimiento en cm/seg
          }
          else {                                                              // Si el tiempo transcurrido fuese cero , asignamos velocidad = 0
                  velocidad = 0;
          }

          if (bluetooth.available() > 0){                                     // En caso de recibir algun mensaje por el canal bluetooth ....
              mensaje = bluetooth.readString();          
                  if (mensaje == "contador"){                                         // Si recibimos mensaje "CONTADOR" ....
                      bluetooth.println(contador);                                    // enviamos al movil el valor de la variable contador
                  }
                  else if (mensaje == "separación"){                                  // Si recibimos el mensaje "SEPARACION" ...
                      bluetooth.println(distancia);                                   // enviamos al movil el valor actual de la separacion del robot a la pared
                  }
                  else if (mensaje == "distancia recorrida"){                         // Si recibimos el mensaje "DISTANCIA RECORRIDA" ...
                      bluetooth.println(distancia_recorrida);                         // enviamos al movil el valor de la distancia recorrida por el robot
                  }
                  else if (mensaje == "velocidad"){                                   // Si recibimos el mensaje "VELOCIDAD" ...
                      bluetooth.println(velocidad);                                   // enviamos al movil el valor de la velocidad del robot 
                  }
                  else if (mensaje == "aceleración"){                                 // Si recibimos el mensaje "ACELERACION" ...
                      bluetooth.println(Aceleracion);                                 // enviamos al movil el valor en grados de avance de los servos de las ruedas ( 0º - 90º)
          }
          }
          break;
          
  case 1:                                                                     // Si estado = 1 entonces ....
          avanzar(RuedaDrcha,RuedaIzqda,Va_derecha,Va_izquierda);             // Ponemos en marcha hacia delante el robot a la velocidad prefijada con el potenciometro
          distancia = ultrasonidos.read();                                    // Leemos el valor de distancia a la pared con el sensor US
          T_final = millis();                                                 // Medimos el instante en miliseg de la lectura de distancia
          delay(50);                                                          // Dejamos pasar 50 milisec entre medidas
          break;


}
  
  /****************************************                       FIN DEL PROGRAMA                   ********************************/

}
