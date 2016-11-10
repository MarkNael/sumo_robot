#include <DCMotor.h>
#include <IRremote.h>
#include <Ping.h>

/**
	Sumo de robots
	Expocarreras 2016
	Integrantes:
		Macula, Alejandro
		Wieilly, Alan
		Amaro, Marcos
		Diaz Burcet, Nehuen
*/

DCMotor motorIzquierdo(M0_EN, M0_D0, M0_D1);
DCMotor motorDerecho(M1_EN, M1_D0, M1_D1);
IRrecv codigoControlIR(A0);
PingSensor ping(A3);
float sensorDerecho, sensorIzquierdo, sensorTrasero;
int velocidadMaxima = 90;
int velocidadMedia = 75;
int velocidadBaja = 55;
int distanciaMaximaAtaque = 35; // distancia maxima a la que va a detectar al otro objetivo dentro de la pista
int frecuenciaBusquedaContrincante = 70;  // es la frecuencia ( cant de ciclos ) con la que va a escanear la pista para buscar al contrincante
int tiempoGiroBusquedaContrincante = 400;  // es el tiempo ( cant de ciclos ) con que va a escanear la pista para buscar al contrincante
int tiempoGiro90Grados = 1250;   // cantidad de ciclos aprox que le lleva al robot girar 90 grados
int tiempoGiro180Grados = 2500;  // cantidad de ciclos aprox que le lleva al robot girar 180 grados
int tiempoGiro360Grados = 5000;  // cantidad de ciclos aprox que le lleva al robot girar 360 grados
int refraccionColorBlancoDerecho = 301;    //valor MINIMO que puede arrojar el sensor derecho al estar sobre el color blanco
int refraccionColorBlancoIzquierdo = 301;  //valor MINIMO que puede arrojar el sensor izquierdo al estar sobre el color blanco
int refraccionColorBlancoTrasero = 201;   //valor MINIMO que puede arrojar el sensor trasero al estar sobre el color blanco
int refraccionColorNegroDerecho = 300;   //valor MAXIMO que puede arrojar el sensor derecho al estar sobre el color negro
int refraccionColorNegroIzquierdo = 300; //valor MAXIMO que puede arrojar el sensor izquierdo al estar sobre el color negro
int refraccionColorNegroTrasero = 200;   //valor MAXIMO que puede arrojar el sensor trasero al estar sobre el color negro
int refraccionFueraDeLaPistaTrasero = 20;   //valor MAXIMO que puede arrojar el sensor trasero al salir de la pista
int ledRojo = 11;      //pin de la placa a la que esta conectado el led rojo
int ledAzul = 10;      //pin de la placa a la que esta conectado el led azul
int ledVerde = 9;      //pin de la placa a la que esta conectado el led verde
int contadorGeneral = 0;  //contador general de ciclos para estrategias internas
boolean flagSaliendoDelBorde = 0;   //indica si el robot esta moviendose por la pista o esta retrocediendo/girando para salir del limite
int velocidadActiva = velocidadBaja;
String estrategiaActiva = "pasivo";
int tiempoDemoraInicio=5000; //tiempo que va a demorar en iniciar el robot

void setup() {
    pinMode(ledRojo, OUTPUT);
    pinMode(ledAzul, OUTPUT);
    pinMode(ledVerde, OUTPUT);

    pinMode(25, INPUT);
    digitalWrite(25, HIGH);
    motorIzquierdo.setClockwise(false);
    motorDerecho.setClockwise(false); // le digo que este motor gira en sentido inverso, asi evito ponerle valor negativo para que avance
    Serial.begin(115200); //seteo la placa para que trabaje a alta frecuencia por el sensor infrarrojo

    for(int i=0; i<tiempoDemoraInicio; i++) { //espero 5 segundos para iniciar
        analogWrite(ledVerde,  255);
        analogWrite(ledAzul,  255);
        analogWrite(ledRojo,  255);
        delay(1);
    }
    analogWrite(ledVerde,  0);
    analogWrite(ledAzul,  0);
    analogWrite(ledRojo,  0);
}

/*
	El ciclo continuo del robot. Aquí se lee constantemente la orden dada desde el control remoto.
	Según la orden especificada, se cambiará la velocidad del robot o aplicará una estrategia.
	En cualquier caso, el robot estará moviendose.
*/
void loop() {

    sensorDerecho = analogRead(1);
    sensorIzquierdo = analogRead(2);
    sensorTrasero = analogRead(4);

    int codigoControlRemoto = codigoControlIR.getIRRemoteCode();

    switch (codigoControlRemoto) {
    case 1:
        velocidadActiva = velocidadMaxima;
        break;
    case 2:
        velocidadActiva = velocidadMedia;
        break;
    case 3:
        velocidadActiva = velocidadBaja;
        break;
    case 7:
        estrategiaActiva = "ataque";
        break;
    case 8:
        estrategiaActiva = "defensa";
        break;
    case 9:
        estrategiaActiva = "pasivo";
        break;
    }

    moverse(velocidadActiva, estrategiaActiva);
}

/*
 *  Parametros: velocidad a la que va a funcionar
 *  Comportamiento: el robot avanzara indefinidamente
 */
void avanzar(int velocidad) {
    motorIzquierdo.setSpeed(velocidad);
    motorDerecho.setSpeed(velocidad);
}

/*
 *  Parametros: velocidad a la que va a funcionar
 *  Comportamiento: el robot retrocedera indefinidamente
 */
void retroceder(int velocidad) {
    motorIzquierdo.setSpeed(-velocidad);
    motorDerecho.setSpeed(-velocidad);
}

/*
 *  Parametros: null
 *  Comportamiento: el robot frenara a 0
 */
void frenar() {
    motorIzquierdo.brake();
    motorDerecho.brake();
}


/*
 *  Parametros: velocidad a la que va a funcionar
 *  Comportamiento: asumiendo que esta funcion se llamara siempre que estemos en el limite de la pista,
 *    retrocedemos unos cm y giramos 180 grados en sentido horario
 */
boolean girarDerecha(int velocidad, int tiempo) {
    int tiempoRestante = tiempo;

    while((tiempoRestante>0) && (sobreLineaBlanca() != 3) && (sensorTrasero>refraccionFueraDeLaPistaTrasero)) {
        motorIzquierdo.setSpeed(-velocidad);
        motorDerecho.setSpeed(0);
        tiempoRestante--;
    }
    if((tiempoRestante>0)) {
        frenar();
        while((tiempoRestante>0) && (sensorTrasero>refraccionFueraDeLaPistaTrasero)) {
            motorIzquierdo.setSpeed(0);
            motorDerecho.setSpeed(velocidad);
            tiempoRestante--;
        }
    }
    frenar();
    if(tiempoRestante == 0 ) return true;
    else return false;
}


/*
 *  Parametros: velocidad a la que va a funcionar
 *  Comportamiento: asumiendo que esta funcion se llamara siempre que estemos en el limite de la pista,
 *    retrocedemos unos cm y giramos 180 grados en sentido antihorario
 */
boolean girarIzquierda(int velocidad, int tiempo) {
    int tiempoRestante = tiempo;

    while((tiempoRestante>0) && (sobreLineaBlanca() != 3) && (sensorTrasero>refraccionFueraDeLaPistaTrasero)) {
        motorIzquierdo.setSpeed(0);
        motorDerecho.setSpeed(-velocidad);
        tiempoRestante--;
    }
    if((tiempoRestante>0) && (sensorTrasero>refraccionFueraDeLaPistaTrasero)) {
        frenar();
        while((tiempoRestante>0) && (sensorTrasero>refraccionFueraDeLaPistaTrasero)) {
            motorIzquierdo.setSpeed(velocidad);
            motorDerecho.setSpeed(0);
            tiempoRestante--;
        }
    }
    frenar();
    if(tiempoRestante == 0 ) return true;
    else return false;
}


/*
 *  Parametros: velocidad a la que va a funcionar
 *  Comportamiento: asumiendo que esta funcion se llamara siempre que estemos en el limite de la pista,
 *    retrocedemos unos cm y giramos 180 grados en sentido antihorario
 */
void girarSobreEje(int velocidad, int tiempo) {
    int i=0;
    if(!tiempo)tiempo=tiempoGiro360Grados;

    while((i< tiempo) && (sobreLineaBlanca() == 0)) {
        motorIzquierdo.setSpeed(-velocidad); //gira en sentido inverso
        motorDerecho.setSpeed(velocidad);
        i++;
    }
    frenar();

}


/*
 *  Parametros: null
 *  Return: 4 en caso de estar parado con ambas ruedas sobre una linea blanca, 1 en caso de estar parado solo con la rueda izquierda,
 *    2 en caso de estarlo solo con la derecha y 3 en caso de estar con la trasera, 0 en caso contrario.
 */
boolean sobreLineaBlanca() {

    sensorDerecho = analogRead(1);
    sensorIzquierdo = analogRead(2);
    sensorTrasero = analogRead(4);

    if((sensorDerecho>refraccionColorBlancoDerecho) && (sensorIzquierdo>refraccionColorBlancoIzquierdo)) {
        frenar();
        return 4;
    }
    else if((sensorDerecho>refraccionColorBlancoDerecho)) {
        frenar();
        return 1;
    }
    else if((sensorIzquierdo>refraccionColorBlancoIzquierdo)) {
        frenar();
        return 2;
    }
    else if((sensorTrasero>refraccionColorBlancoTrasero)) {
        frenar();
        return 3;
    }
    else {
        return 0;
    }

}

/*
 *  Parametros: velocidad a la que va a funcionar
 *  Comportamiento: el robot va a girar sobre su propio eje hasta encontrar algun obstaculo,
 *    en ese momento se detendra y quedara apuntado en su direccion listo para atacar
 */
boolean girarHastaEncontrarDireccionObjetivo(int velocidad) {
    int tiempoRestante = frecuenciaBusquedaContrincante;

    while((tiempoRestante>0) && (!objetivoCerca()) && (sobreLineaBlanca() != 1) && (sobreLineaBlanca() != 2)) {
        motorIzquierdo.setSpeed(-velocidad);
        motorDerecho.setSpeed(velocidad);
        tiempoRestante--;
    }
    frenar();
    sensorDerecho = analogRead(1);
    sensorIzquierdo = analogRead(2);
    if (objetivoCerca() && (sobreLineaBlanca() != 1) && (sobreLineaBlanca() != 2)) {
        return true;
    }
    else {
        return false;
    }
}

/*
 *  Parametros: null
 *  Return: TRUE en caso de tener un objetivo a menos de 18 cm en la direccion del robot, FALSE en caso contrario.
 */
boolean objetivoCerca() {
    return(ping.measureCM() < distanciaMaximaAtaque);
}

/*
 *  Parametros: velocidad a la que va a funcionar, estrategia: "ataque", "defensa"
 *  Comportamiento: el robot va a avanzar en linea recta mientras nos estemos acercando al objetivo detectado,
 *    en caso de alejarnos respecto a la ultima distancia grabada sale de la funcion
 */
void moverse(int velocidad, String estrategia) {
    contadorGeneral++;
    int sobreLineaBlancaFlag = 0;
    sensorDerecho = analogRead(1);
    sensorIzquierdo = analogRead(2);
    sensorTrasero = analogRead(4);

    if(((sensorDerecho<refraccionColorNegroDerecho)  && (sensorIzquierdo<refraccionColorNegroIzquierdo) && !flagSaliendoDelBorde)) {
        if(objetivoCerca() && estrategiaActiva != "pasivo") {
            if (estrategiaActiva == "ataque") {
                analogWrite(ledRojo,  255);
                analogWrite(ledVerde,  0);
                analogWrite(ledAzul,  255);
                avanzar(100);
            }
            else if( estrategia == "defensa") {
                analogWrite(ledAzul,  255);
                analogWrite(ledVerde,  255);
                analogWrite(ledRojo,  0);
                girarDerecha(velocidadMaxima, tiempoGiro90Grados);
            }
        }
        else {
            analogWrite(ledVerde,  255);
            analogWrite(ledAzul,  0);
            analogWrite(ledRojo,  0);
            if (((contadorGeneral % frecuenciaBusquedaContrincante) == 0) && estrategiaActiva == "ataque") {
                girarHastaEncontrarDireccionObjetivo(velocidad);
            }
            else {
                avanzar(velocidad);
            }
        }
    }
    else {
        flagSaliendoDelBorde = true;
        sobreLineaBlancaFlag = sobreLineaBlanca();
        analogWrite(ledRojo,  255);
        analogWrite(ledVerde,  0);
        analogWrite(ledAzul,  0);
        //contraresto la inercia del robot para que no se vaya de la pista
        retroceder(70);
        delay(400);
        frenar();
        //

        int tiempo;
        int flag = random(0,2);

        if( flag == 0 ) tiempo = tiempoGiro90Grados;
        else if ( flag == 1 ) tiempo = tiempoGiro180Grados;

        if (sobreLineaBlancaFlag == 1) { // con la rueda derecha sobre la linea blanca
            if(velocidad > 61 ) {
                retroceder(100);
                delay(150);
                if(girarIzquierda((velocidad*2)/3, tiempo)) {
                    flagSaliendoDelBorde = false;
                }
            }
            else {
                if(girarIzquierda(velocidad, tiempo)) {
                    flagSaliendoDelBorde = false;
                }
            }
            frenar();
        }
        else if (sobreLineaBlancaFlag == 2) { // con la rueda izquierda sobre la linea blanca
            if(velocidad > 61 ) {
                retroceder(100);
                delay(150);
                if(girarIzquierda((velocidad*2)/3, tiempo)) {
                    flagSaliendoDelBorde = false;
                }
            }
            else {
                if(girarIzquierda(velocidad, tiempo)) {
                    flagSaliendoDelBorde = false;
                }
            }
            frenar();
        }
        else {
            // si por algun motivo sigo tocando la linea blanca retrocedo para asegurarme que no se va a salir
            retroceder(100);
            delay(300);
            frenar();
            //
            if(velocidad > 61 ) {
                girarSobreEje((velocidad*2)/3, tiempoGiro90Grados);
            }
            else girarSobreEje(velocidad, tiempoGiro90Grados);
            frenar();
            flagSaliendoDelBorde = false;
        }
    }
}
