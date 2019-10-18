import math
import minimization
import plot
import numpy as np

def VelocityProfile(vs,ve,Fmax,Amax,Jerk,L,period):
    time_list =[]
    t1 = 0
    t2 = 0
    t4 = 0
    t5 = 0
    t6 = 0

    acc = FmaxTime(Fmax,vs,Jerk,Amax/Jerk)
    dec = FmaxTime(Fmax,ve,Jerk,Amax/Jerk)

    t1 = acc[0]
    t2 = acc[1]
    sa = acc[2]
    t5 = dec[0]
    t6 = dec[1]
    sd = dec[2]

    if sa+sd <= L:
        #可以达到最大速度，且可以维持最大速度加工一段时间t4
        t4 = (L-sa-sd)/Fmax
    else:
        #达不到Fmax
        if vs == ve:
            #先令t2=0,计算t1,求解一元三次方程：J*t1^3 + 2*vs*t1 = L/2
            temp = Cubic_Equation(Jerk,0,2*vs,-0.5*L)
            if temp> Amax/Jerk:
                t1 = Amax/Jerk
                #求解一元二次方程:J*t1*t2^2 + (2*vs+3*J*t1^2)*t2+2*J*t1^3+4*vs*t1-L=0
                t2 = Quadratic_Equation(Jerk*t1,2*vs+3*Jerk*math.pow(t1,2),2*Jerk*math.pow(t1,3)+4*vs*t1-L)
                t5 = t1
                t6 = t2
            else:
                t2 = 0
                t6 = 0
                t1 = temp
                t5 = temp
        else:
            args = (vs,Jerk,Fmax,L,period)
            args1=(Fmax,ve,Amax,Jerk,vs,period)
            x0 = np.array((1,0,0,0))
            number_list = minimization.optimization(args,args1,x0)
            t1 = number_list[0]*period
            t2 = number_list[1]*period
            t5 = number_list[2]*period
            t6 = number_list[3]*period
    time_list.append(t1)
    time_list.append(t2)
    time_list.append(t4)
    time_list.append(t5)
    time_list.append(t6)

    res = TimePeriod(time_list,period)
    time_value = res[0]
    time_int = res[1]

    jerk = JerkProfile(time_value,time_int,period,vs,ve,L,Jerk)
    time_int = jerk[2]
    ans = PlotCurve(time_int,jerk[0],jerk[1],period,vs)

    
    return ans,time_int,jerk
    
    
def TimePeriod(time_list,period):
    T1 = 0 if time_list[0] == 0 else math.floor(time_list[0]/period)+1
    T2 = math.floor(time_list[1]/period)
    T4 = math.floor(time_list[2]/period)
    T5 = 0 if time_list[3] == 0 else math.floor(time_list[3]/period)+1
    T6 = math.floor(time_list[4]/period)

    if T1 ==0:
        T2 = 0
    if T5 == 0:
        T6 = 0

    t1 = T1*period
    t2 = T2*period
    t4 = T4*period
    t5 = T5*period
    t6 = T6*period

    time =[]
    time.append(t1)
    time.append(t2)
    time.append(t4)
    time.append(t5)
    time.append(t6)

    time_int = []
    time_int.append(T1)
    time_int.append(T2)
    time_int.append(T4)
    time_int.append(T5)
    time_int.append(T6)
    return time,time_int



def JerkProfile(time_list,time_int,period,vs,ve,L,Jerk):
    t1 = time_list[0]
    t2 = time_list[1]
    t4 = time_list[2]
    t5 = time_list[3]
    t6 = time_list[4]

    Jerka = Jerk
    Jerkd = Jerk

    if t1 != 0 or t5 != 0:
        res = Binary_Equation(t1,t2,t4,t5,t6,vs,ve,L)
        Jerka = res[0]
        Jerkd = res[1]
        while(Jerka>Jerk or Jerkd>Jerk):
            t2 = t2+ period
            t6 = t6 + period
            time_int[1] = time_int[1]+1
            time_int[4] = time_int[4]+1
            res = Binary_Equation(t1,t2,t4,t5,t6,vs,ve,L)
            Jerka= res[0]
            Jerkd=res[1]
        
    return Jerka,Jerkd,time_int         

def Binary_Equation(t1,t2,t4,t5,t6,vs,ve,L):
    a = t1*(t1+t2)
    b = -t5*(t5+t6)
    c = ve-vs
    d = t1*(2*t1+t2)*(t1+t2)/2 + t1*t4*(t1+t2)
    e = t5*(2*t5+t6)*(t5+t6)/2
    f = L - ve*(2*t5+t6) - vs*(2*t1+t2+t4)
    if b==0 and e==0:
        y = 0
        x = f/d
    elif a == 0 and d == 0:
        x = 0
        y = f/e
    else:
        x = (c*e - b*f)/(a*e - b*d)
        y = (c*d - a*f)/(b*d - a*e)
    return x,y

def PlotCurve(time_list,Jerka,Jerkd,period,vs):
    velocity=[]
    acceleration=[]
    jerk = []
    displacement=[]
    time=[]
    time.append(0)
    velocity.append(vs)
    acceleration.append(0)
    jerk.append(0)
    displacement.append(0)
    tt = 0
    vv = vs
    ss = 0

    if time_list[0] != 0:
        for i in range(time_list[0]):
            t = (i+1)*period
            tt = tt + period
            time.append(tt)
            a = Jerka*t
            acceleration.append(a)
            jerk.append(Jerka)
            v = VelocityValue(vs,Jerka,t,0)
            velocity.append(v)
            s = DisplacementValue(vs,Jerka,t,0)
            displacement.append(s)
        vv = VelocityValue(vs,Jerka,time_list[0]*period,0)
        ss = DisplacementValue(vs,Jerka,time_list[0]*period,0)

        if time_list[1] != 0:
            for i in range(time_list[1]):
                t = (i+1)*period
                tt = tt + period
                time.append(tt)
                a = Jerka*time_list[0]*period
                acceleration.append(a)
                jerk.append(0)
                v = VelocityValue(vv,0,t,a)
                velocity.append(v)
                s = ss + DisplacementValue(vv,0,t,a)
                displacement.append(s)
            ss = ss + DisplacementValue(vv,0,time_list[1]*period,a)
            vv = VelocityValue(vv,0,time_list[1]*period,a)

        for i in range(time_list[0]):
            t = (i+1)*period
            tt = tt + period
            time.append(tt)
            a = Jerka*time_list[0]*period - Jerka*t
            acceleration.append(a)
            jerk.append(-Jerka)
            v = VelocityValue(vv,-Jerka,t,Jerka*time_list[0]*period)
            s = ss + DisplacementValue(vv,-Jerka,t,Jerka*time_list[0]*period)
            velocity.append(v)
            displacement.append(s)
        ss = ss + DisplacementValue(vv,-Jerka,time_list[0]*period,Jerka*time_list[0]*period)
        vv = VelocityValue(vv,-Jerka, time_list[0]*period, Jerka*time_list[0]*period)
    if time_list[2] != 0:
        for i in range(time_list[2]):
            t = (i+1)*period
            tt = tt + period
            time.append(tt)
            acceleration.append(0)
            jerk.append(0)
            velocity.append(vv)
            s = ss + vv*t
            displacement.append(s)
        ss = ss + vv*time_list[2]*period

    if time_list[3] != 0:
        for i in range(time_list[3]):
            t = (i+1)*period
            tt = tt + period
            time.append(tt)
            a = -Jerkd*t
            acceleration.append(a)
            jerk.append(-Jerkd)
            v = VelocityValue(vv,-Jerkd,t,0)
            velocity.append(v)
            s = ss + DisplacementValue(vv,-Jerkd,t,0)
            displacement.append(s)
        ss = ss + DisplacementValue(vv,-Jerkd,time_list[3]*period,0)
        vv = VelocityValue(vv,-Jerkd,time_list[3]*period,0)

        if time_list[4] != 0:
            for i in range(time_list[4]):
                t = (i+1)*period
                tt = tt +period
                time.append(tt)
                a = -Jerkd*time_list[3]*period
                acceleration.append(a)
                jerk.append(0)
                v = VelocityValue(vv,0,t,a)
                s = ss + DisplacementValue(vv,0,t,a)
                velocity.append(v)
                displacement.append(s)
            ss = ss + DisplacementValue(vv,0,time_list[4]*period,a)
            vv = VelocityValue(vv,0,time_list[4]*period,a)

        for i in range(time_list[3]):
            t = (i+1)*period
            tt = tt + period
            time.append(tt)
            a = -Jerkd*time_list[3]*period + Jerkd*t
            acceleration.append(a)
            jerk.append(Jerkd)
            v = VelocityValue(vv,Jerkd,t,-Jerkd*time_list[3]*period)
            s = ss + DisplacementValue(vv,Jerkd,t,-Jerkd*time_list[3]*period)
            velocity.append(v)
            displacement.append(s)
    return time,velocity,acceleration,jerk,displacement


def VelocityValue(vs,Jerk,t,A):
    v = vs + A*t + 0.5*Jerk*math.pow(t,2)
    return v

def DisplacementValue(vs,Jerk,t,A):
    s = vs*t + 0.5*A*math.pow(t,2) + Jerk*math.pow(t,3)/6
    return s
    
    
            
#计算加速到Fmax所需的时间
def FmaxTime(Fmax,vs,Jerk,tmax):
    t = math.sqrt((Fmax-vs)/Jerk)
    t2 = 0
    s = 0
    if t> tmax:
        t2 = (Fmax - vs)/(Jerk*tmax) - tmax
        t = tmax
    s = Displacement(vs,t,t2,Jerk)
    return t,t2,s

#计算t1,t2后的位移
def Displacement(vs,t1,t2,Jerk):
    displacement = vs*(2*t1+t2) + 0.5*Jerk*t1*(2*t1+t2)*(t1+t2)
    return displacement

def Cubic_Equation(a,b,c,d):
    #J*t1^3 + 2*vs*t1 = L/2 这个一元三次方程的判别式D>0，因此该方程有唯一实数根
    A = math.pow(b,2) -3*a*c
    B = b*c - 9*a*d
    C = math.pow(c,2) - 3*b*d
    D = math.pow(B,2) - 4*A*C
    Y1 = 1.5*a*(-B+math.sqrt(D))
    Y2 = 1.5*a*(-B-math.sqrt(D))
    if Y1 < 0:
        if Y2 < 0:
            x1 = (math.pow(-Y1,1/3)+math.pow(-Y2,1/3))/(3*a)
        else:
            x1 = (math.pow(-Y1,1/3)-math.pow(Y2,1/3))/(3*a)
    else:
        if Y2 < 0:
            x1 = (-math.pow(Y1,1/3)+math.pow(-Y2,1/3))/(3*a)
        else:
            x1 = (-math.pow(Y1,1/3)-math.pow(Y2,1/3))/(3*a)
    return x1

def Quadratic_Equation(a,b,c):
    D = math.pow(b,2)-4*a*c
    x = (-b + math.sqrt(D))/(2*a)
    return x





#VelocityProfile(vs,ve,Fmax,Amax,Jerk,L,period)
#JerkProfile(time_list,period,vs,ve,L,Jerk)
period = 0.002
#time_list = [13*period,9*period,61*period,13*period,11*period]
#ans = JerkProfile(time_list,period,10,5,20,80000)
#print(ans)

ans = VelocityProfile(10,35,80,2000,80000,5,period)
print(ans[1])
print(ans[2])
plot.plotting(ans[0][0],ans[0][1],"time(sec)","velocity(mm/sec)","Velocity Profile","Velocity")
plot.plotting(ans[0][0],ans[0][2],"time(sec)","acceleration(mm/sec2)","Acceleration Profile","Acceleration")
plot.plotting(ans[0][0],ans[0][3],"time(sec)","jerk(mm/sec3)","Jerk Profile","Jerk")
plot.plotting(ans[0][0],ans[0][4],"time(sec)","displacement(mm)","Displacement Profile","Displacement")
