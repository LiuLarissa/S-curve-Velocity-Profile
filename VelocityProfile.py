import Methods
import Classes as cl
import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt


motions=["G00","G0","G01","G1","G02","G2","G03","G3"]
straight=["G00","G0","G01","G1"]
circular=["G02","G2","G03","G3"]

class TimeCycle:
    def __init__(self,t1,t2,t4,t5,t6):
        self.t1=t1
        self.t2=t2
        self.t4=t4
        self.t5=t5
        self.t6=t6

class PeriodCycle:
    def __init__(self,n1,n2,n4,n5,n6):
        self.n1=n1
        self.n2=n2
        self.n4=n4
        self.n5=n5
        self.n6=n6

def VelocityPlanning(path,acc,jerk,period,chorderror,maxfeed):
    commands = Methods.ReadFile(path)

    global Acc
    global Jerk
    global Period
    global ChordError
    global MaxFeed
    global feedrate_list
    global acceleration_list
    global jerk_list
    global displacement_list
    global time_list

    Acc=acc
    Jerk=jerk
    Period=period
    ChordError=chorderror
    MaxFeed=maxfeed
    feedrate_list=[]
    acceleration_list=[]
    jerk_list=[]
    displacement_list=[]
    time_list=[]
    
    feedrate_list.append(0)
    acceleration_list.append(0)
    jerk_list.append(0)
    displacement_list.append(0)
    time_list.append(0)
    '''
    times=FeedLimitation(commands)
    for i in times:
        print(np.round(i.t1,3), np.round(i.t2),np.round(i.t4), np.round(i.t5),np.round(i.t6))
    '''
    ans=FeedLimitation(commands)
    feedrate=ans[0]
    displacement=ans[1]
    #times=[]

    for i in range(1,len(feedrate)):
        ans=ScheduleSingleBlock(feedrate[i-1],feedrate[i],displacement[i],50)
        feedrate[i]=ans if ans<=MaxFeed/60 else MaxFeed/60



def FeedLimitation(commands):
    feedrate=[]
    displacement=[]
    feedrate.append(0)
    displacement.append(0)
    velocity=0
    first=commands[0]
    second=commands[1]
    third=commands[2]
    times=[]
 
    for i in range(3,len(commands)):
        if second.motion_type in motions and third.motion_type in motions:
            if second.feedrate==0 and (second.motion_type=="G00" or second.motion_type=="G0"):
                second.feedrate=MaxFeed
            
            if second.motion_type in straight:
                if third.motion_type in straight:
                    cos=Methods.IncludedAngle(first.position, second.position, third.position, second.plane)
                    if cos==2:
                        velocity=0
                    elif cos==1:
                        velocity=min(second.feedrate/60,third.feedrate/60)
                    else:
                        velocity=min(second.feedrate/60,third.feedrate/60,Acc*Period/(1-cos))   
                elif third.motion_type in circular:
                    center=cl.position(second.position.x+third.position.i,
                                        second.position.y+third.position.j,
                                        second.position.z+third.position.k,0,0,0,third.position.r)
                    endpoint = Methods.TangentPoint(second.position,center,5,third.motion_type,second.plane)
                    cos=Methods.IncludedAngle(first.position,second.position,endpoint,second.plane)
                    if cos == 2:
                        velocity=0
                    elif cos==1:
                        velocity=min(second.feedrate/60,third.feedrate/60,2*np.sqrt(ChordError*(2*center.r-ChordError))/Period)
                    else:
                        velocity=min(second.feedrate/60,third.feedrate/60,Acc*Period/(1-cos),2*np.sqrt(ChordError*(2*center.r-ChordError))/Period)
                
                length=Methods.LinearLength(first.position,second.position)
                velocity=CheckFeedrate(feedrate[-1],np.round(velocity,6),length)
            elif second.motion_type in circular:
                center=cl.position(first.position.x+second.position.i,first.position.y+second.position.j,
                                    first.position.z+second.position.k,0,0,0,second.position.r)
                chord_temp=2*np.sqrt(ChordError*(2*center.r-ChordError))/Period
                endpoint=Methods.TangentPoint(second.position,center,-5,second.motion_type,second.plane)
                
                if third.motion_type in straight:
                    cos=Methods.IncludedAngle(endpoint,second.position,third.position,second.plane)
                    if cos==2:
                        velocity=0
                    elif cos==1:
                        velocity=min(second.feedrate/60,third.feedrate/60)
                    else:
                        velocity=min(second.feedrate/60,third.feedrate/60,chord_temp,Acc*Period/(1-cos))
                elif third.motion_type in circular:
                    center1=cl.position(second.position.x+third.position.i,second.position.y+third.position.j,second.position.z+third.position.k,
                                        0,0,0,third.position.r)
                    endpoint1=Methods.TangentPoint(second.position,center1,5,third.motion_type,third.plane)
                    cos=Methods.IncludedAngle(endpoint,second.position,endpoint1,third.plane)
                    if cos==2:
                        velocity=0
                    elif cos==1:
                        velocity=min(second.feedrate/60,third.feedrate/60,chord_temp,2*np.sqrt(ChordError*(2*center1.r-ChordError))/Period)
                    else:
                        velocity=min(second.feedrate/60,third.feedrate/60,chord_temp,Acc*Period/(1-cos),
                                    2*np.sqrt(ChordError*(2*center1.r-ChordError))/Period)
                length=Methods.CircularLength(first.position,second.position,center,second.motion_type,second.plane)
                velocity=CheckFeedrate(feedrate[-1],np.round(velocity,6),length)
            
            #time=ScheduleSingleBlock(feedrate[-1],velocity,length,second.feedrate/60)
            #times.append(time)
            feedrate.append(velocity)
            displacement.append(length)

        first=second
        second=third
        third=commands[i]   
    
    return feedrate,displacement

  
def ScheduleSingleBlock(vs,ve,length,maxfeed):
    t1=Acc/Jerk
    t2=(maxfeed-vs)/(Jerk*t1)-t1
    if t2<0:
        t2=0
        t1=np.sqrt((maxfeed-vs)/Jerk)
    la=vs*(2*t1+t2)+0.5*Jerk*t1*(2*t1+t2)*(t1+t2)
    t5=Acc/Jerk
    t6=(maxfeed-ve)/(Jerk*t5)-t5
    if t6<0:
        t6=0
        t5=np.sqrt((maxfeed-ve)/Jerk)
    ld=ve*(2*t5+t6)+0.5*Jerk*t5*(2*t5+t6)*(t5+t6)
    
    if la+ld<=length:
        t4=(length-la-ld)/maxfeed
    else:
        t4=0
        if t2==0:
            if t6==0:
                fun=lambda t:(vs*2*t[0]+Jerk*t[0]**3+ve*2*t[1]+Jerk*t[1]**3-length)**2
                bnds=((0,Acc/Jerk),(0,Acc/Jerk))
                cons=({'type':'eq','fun':lambda t: vs+Jerk*t[0]**2-ve-Jerk*t[1]**2})
                res=minimize(fun,(t1,t5),method='SLSQP',bounds=bnds,constraints=cons)
                t1=res.x[0]
                t5=res.x[1]
            else:
                fun=lambda t:(vs*2*t[0]+Jerk*t[0]**3+ve*(2*t[1]+t[2])+0.5*Jerk*t[1]*(2*t[1]+t[2])*(t[1]+t[2])-length)**2
                bnds=((0,Acc/Jerk),(0,Acc/Jerk),(0,None))
                cons=({'type':'eq','fun':lambda t:vs+Jerk*t[0]**2-ve-Jerk*t[1]*(t[1]+t[2])},
                      {'type':'ineq','fun':lambda t:maxfeed-vs-Jerk*t[0]**2})
                res=minimize(fun,(t1,t5,t6),method="SLSQP",bounds=bnds,constraints=cons)
                t1=res.x[0]
                t5=res.x[1]
                t6=res.x[2]
        else:
            if t6==0:
                fun=lambda t:(vs*(2*t[0]+t[1])+0.5*Jerk*t[0]*(2*t[0]+t[1])*(t[0]+t[1])+ve*2*t[2]+Jerk*t[2]**3-length)**2
                bnds=((0,Acc/Jerk),(0,None),(0,Acc/Jerk))
                cons=({'type':'eq','fun':lambda t:vs+Jerk*t[0]*(t[0]+t[1])-ve-Jerk*t[2]**2},
                      {'type':'ineq','fun':lambda t:maxfeed-ve-Jerk*t[2]**2})
                res=minimize(fun,(t1,t2,t5),method='SLSQP',bounds=bnds,constraints=cons)
                t1=res.x[0]
                t2=res.x[1]
                t5=res.x[2]
            else:
                fun=lambda t:(vs*(2*t[0]+t[1])+0.5*Jerk*t[0]*(2*t[0]+t[1])*(t[0]+t[1])+ve*(2*t[2]+t[3])+0.5*Jerk*t[2]*(2*t[2]+t[3])*(t[2]+t[3])-length)**2
                bnds=((0,Acc/Jerk),(0,None),(0,Acc/Jerk),(0,None))
                cons=({'type':'eq','fun':lambda t:vs+Jerk*t[0]*(t[0]+t[1])-ve-Jerk*t[2]*(t[2]+t[3])},
                      {'type':'ineq','fun':lambda t:maxfeed-vs-Jerk*t[0]*(t[0]+t[1])})
                res=minimize(fun,(t1,t2,t5,t6),method='SLSQP',bounds=bnds,constraints=cons)
                t1=res.x[0]
                t2=res.x[1]
                t5=res.x[2]
                t6=res.x[3]
    time=TimeCycle(t1,t2,t4,t5,t6)
    cycle=PeriodCycle(np.floor(time.t1/Period),np.floor(time.t2/Period),np.floor(time.t4/Period),np.floor(time.t5/Period),np.floor(time.t6/Period))
    ans=CalculateFeedrate(cycle,length,vs,ve)
    return ans
    #return cycle


def CalculateFeedrate(cycle,length,vs,ve):
    #Period=0.001
    if cycle.n1==0:
        Jerka=0
        a=1
        b=cycle.n5*(cycle.n5+cycle.n6)*Period**2
        c=vs
        m=(2*cycle.n5+cycle.n6)*Period
        n=0.5*cycle.n5*(2*cycle.n5+cycle.n6)*(cycle.n5+cycle.n6)*Period**3
        z=length-vs*cycle.n4*Period
        args=(a,b,c,m,n,z)
        ans=Methods.BinaryEquation(args)
        ve=ans[0]
        Jerkd=ans[1]
    elif cycle.n5==0:
        Jerkd=0
        Jerka=(length-vs*(cycle.n4+2*cycle.n1+cycle.n2)*Period)/((0.5*cycle.n1*(2*cycle.n1+cycle.n2)*(cycle.n1+cycle.n2)+cycle.n1*cycle.n4*(cycle.n1+cycle.n2))*Period**3)
        ve=vs+Jerka*cycle.n1*(cycle.n1+cycle.n2)*Period**2
    else:
        a=cycle.n1*(cycle.n1+cycle.n2)*Period**2
        b=-cycle.n5*(cycle.n5+cycle.n6)*Period**2
        c=ve-vs
        m=0.5*(cycle.n1*(2*cycle.n1+cycle.n2)*(cycle.n1+cycle.n2)+2*cycle.n1*cycle.n4*(cycle.n1+cycle.n2))*Period**3
        n=0.5*cycle.n5*(2*cycle.n5+cycle.n6)*(cycle.n5+cycle.n6)*Period**3
        z=length-vs*(2*cycle.n1+cycle.n2)*Period-ve*(2*cycle.n5+cycle.n6)*Period-vs*cycle.n4*Period
        args=(a,b,c,m,n,z)
        ans=Methods.BinaryEquation(args)
        Jerka=ans[0]
        Jerkd=ans[1]
    #print(Jerka,Jerkd,ve)
    #print(cycle.n1,cycle.n2,cycle.n4,cycle.n5,cycle.n6)

    if cycle.n1 >0:
        feed_last=feedrate_list[-1]
        s_last=displacement_list[-1]
        t_last=time_list[-1]
        for i in range(1,int(cycle.n1)):
            jerk_list.append(Jerka)
            acceleration_list.append(Jerka*i*Period)
            f=feed_last+0.5*Jerka*(i*Period)**2
            feedrate_list.append(f)
            s=s_last+feed_last*i*Period+Jerka*(i*Period)**3/6
            displacement_list.append(s)
            time_list.append(t_last+i*Period)
        if cycle.n2>0:
            feed_last=feedrate_list[-1]
            s_last=displacement_list[-1]
            t_last=time_list[-1]
            for i in range(1,int(cycle.n2)):
                jerk_list.append(0)
                acceleration_list.append(Jerka*cycle.n1*Period)
                f=feed_last+Jerka*cycle.n1*Period*i*Period
                feedrate_list.append(f)
                s=s_last+feed_last*i*Period+0.5*Jerka*cycle.n1*Period*(i*Period)**2
                displacement_list.append(s)
                time_list.append(t_last+i*Period)
        feed_last=feedrate_list[-1]
        s_last=displacement_list[-1]
        t_last=time_list[-1]
        for i in range(1,int(cycle.n1)):
            jerk_list.append(-Jerka)
            acceleration_list.append(Jerka*cycle.n1*Period-Jerka*i*Period)
            f=feed_last+Jerka*cycle.n1*Period*i*Period-0.5*Jerka*(i*Period)**2
            feedrate_list.append(f)
            s=s_last+feed_last*i*Period+0.5*Jerka*cycle.n1*Period*(i*Period)**2-Jerka*(i*Period)**3/6
            displacement_list.append(s)
            time_list.append(t_last+i*Period)
    if cycle.n4>0:
        feed_last=feedrate_list[-1]
        s_last=displacement_list[-1]
        t_last=time_list[-1]
        for i in range(1,int(cycle.n4)):
            jerk_list.append(0)
            acceleration_list.append(0)
            feedrate_list.append(feed_last)
            displacement_list.append(s_last+feed_last*i*Period)
            time_list.append(t_last+i*Period)
    if cycle.n5>0:
        feed_last=feedrate_list[-1]
        s_last=displacement_list[-1]
        t_last=time_list[-1]
        for i in range(1,int(cycle.n5)):
            jerk_list.append(-Jerkd)
            acceleration_list.append(-Jerkd*i*Period)
            f=feed_last-0.5*Jerkd*(i*Period)**2
            feedrate_list.append(f)
            s=s_last+feed_last*i*Period-Jerkd*(i*Period)**3/6
            displacement_list.append(s)
            time_list.append(t_last+i*Period)
        if cycle.n6>0:
            feed_last=feedrate_list[-1]
            s_last=displacement_list[-1]
            t_last=time_list[-1]
            for i in range(1,int(cycle.n6)):
                jerk_list.append(0)
                acceleration_list.append(-Jerkd*cycle.n6*Period)
                f=feed_last-Jerkd*cycle.n6*Period*i*Period
                feedrate_list.append(f)
                s=s_last+feed_last*i*Period-0.5*Jerkd*cycle.n6*Period*(i*Period)**2
                displacement_list.append(s)
                time_list.append(t_last+i*Period)
        feed_last=feedrate_list[-1]
        s_last=displacement_list[-1]
        t_last=time_list[-1]
        for i in range(1,int(cycle.n5)):
            jerk_list.append(Jerkd)
            acceleration_list.append(-Jerkd*cycle.n5*Period+Jerkd*i*Period)
            f=feed_last-Jerkd*cycle.n5*Period*i*Period+0.5*Jerkd*(i*Period)**2
            feedrate_list.append(f)
            s=s_last+feed_last*i*Period-0.5*Jerkd*cycle.n5*Period*(i*Period)**2+Jerkd*(i*Period)**3/6
            displacement_list.append(s)
            time_list.append(t_last+i*Period)
    
    return ve

def CheckFeedrate(vs,ve,length):
    v=np.abs(vs-ve)
    v1=min(vs,ve)
    t1=Acc/Jerk
    t2=v/Acc-t1
    if t2<0:
        t2=0
        t1=np.sqrt(v/Jerk)
    
    length1=v1*(2*t1+t2)+0.5*Jerk*t1*(2*t1+t2)*(t1+t2)
    if length1<=length:
        return ve
    else:
        args=(Jerk,0,2*v1,-length)
        ans=Methods.CubicEquation(args)
        flag=0
        for i in ans:
           if i.imag ==0 and i.real>=0:
               t1=i
               flag=1
        if flag==0:
            return vs
        else:
            if ve>vs:
                return vs+Jerk*t1**2
            else:
                return vs-Jerk*t1**2 

path="D:\\Components1119\\MotionCommands\\MotionCommand_milling#1.txt"
acc=2000
jerk=50000
period=0.001
error=0.001
feed=3000

VelocityPlanning(path,acc,jerk,period,error,feed)

fig,[ax1,ax2]= plt.subplots(2,1,sharex=True)

ax1.plot(time_list, feedrate_list)
ax2.plot(time_list,acceleration_list)
plt.show()








            




