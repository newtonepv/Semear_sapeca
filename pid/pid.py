class Pid:
    def __init__(self, kp: float, ki: float, kd: float):
        self._soma_erro=0
        self._erro_prev=0
        self._updated=False 
        self._kp = kp
        self._ki = ki
        self._kd = kd
    
    def set_kp(self, kp):
        self._kp=kp

    def set_ki(self, ki):
        self._ki=ki
    
    def set_kd(self, kd):
        self._kd=kd

    def get_kp(self):
        return self._kp

    def get_ki(self):
        return self._ki

    def get_kd(self):
        return self._kd
    
    def update(self, sensor:float, sp:float, tempo:float):
        erro = sensor-sp    #calcula o erro

        p=erro*self._kp  #proporcional

        self._soma_erro+=erro*tempo*self._ki  #integral

        d=0
        if(self._updated==True):
            #se nao tiver registrado o erro previo
            d=self._kd*(self._erro_prev-erro)/tempo
        else:
            #deixar claro que registrou o erro previo (so roda a partir do seguinte update)
            self._updated=True

        self._erro_prev=erro #registra o erro previo
        mv = d+p+self._soma_erro
        return mv