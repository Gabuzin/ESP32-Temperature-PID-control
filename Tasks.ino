// Tasks RTOS
void Task_PID(void * pvParameters)
{  
  while(1)
  {
      // PID part
  input = t; // iguala temperatura ao input do PID
  error = setpoint - input;  // Calcular o erro do PID
  myPID.Compute();  // Calcular o valor de controle do PID
  
   if (On==1) // verifica se o sistema esta ligado
  { 
   
   digitalWrite(PIN_FAN,LOW);// Liga ventilador
    
   pd_output= 255- output;
   analogWrite(PIN_PWM, pd_output);  // Enviar o valor do PWM para o sistema
   tempo_cont=0;
  }
    if (On==0) // desliga sistema
  {   digitalWrite(PIN_FAN,HIGH); // Desliga ventilador
     analogWrite(PIN_PWM, 255);
  
  }

  vTaskDelay(300 / portTICK_PERIOD_MS); // define a cada quanto tempo ocorre a task
  }
}

void task_publicaMQTT(void * pvParameters)
{
  while(1){

        if (client.connected())  // verifica conexão com mqtt, caso desconectado chama função para reconectar
    {     
    String dado = String(t);// transforma o valor int em string
    client.publish(Temp_topic ,dado.c_str());// publicando dados
    String dado1 = String(h); // transforma o valor int em string
    client.publish(H_topic ,dado1.c_str());// publicando dados
    String dado2 = String(rpm); // transforma o valor int em string
    client.publish(RPM_topic ,dado2.c_str());// publicando dados
    }// fim do if 

    else{
      ConectaMQTT ();
    }

  vTaskDelay(2000 / portTICK_PERIOD_MS); // define a cada quanto tempo ocorre a task
  }
}

void Task_ler_Sensores(void * pvParameters) // faz leitura dos sensores a cada 2 segundos
{
  while(1)
  {
    t = dht.readTemperature();
    h = dht.readHumidity();    
    vTaskDelay(2000 / portTICK_PERIOD_MS); // define a cada quanto tempo ocorre a task
  }
}

void S_VENT(void * pvParameters)
{
  while(1)
  {
  rpm=count*60/2; // conversão de pulsos por segundo para rpm
  count=0; // zera contador
  
  vTaskDelay(1000 / portTICK_PERIOD_MS); // define a cada quanto tempo ocorre a task
  }
} 
void Tempo_Secagem(void * pvParameters)
{
  while(1)
  {
  tempo_cont++;
   if(tempo_cont==tempo)
   {On=0;
    output=0;
    tempo_cont=0;
    }
  
  vTaskDelay(60000 / portTICK_PERIOD_MS); // define a cada quanto tempo ocorre a task
  }
} 




