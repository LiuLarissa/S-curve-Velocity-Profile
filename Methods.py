import Classes as cl
import numpy as np
import cmath

motions=["G00","G0","G01","G1","G02","G2","G03","G3"]

def ReadFile(path):
    commands=[]
    for line in open(path):
        terms=line.split()
        position = cl.position(float(terms[5]), float(terms[6]), float(terms[7]),float(terms[8]),
                    float(terms[9]),float(terms[10]),float(terms[11]))
        command=cl.motion(terms[1],position,float(terms[0]),float(terms[4]),terms[3])
        commands.append(command)
    
    return commands

# first, second, and third are positions
# compute the included angle between two lines in the moving direction
def IncludedAngle(first,second,third,plane):
    precision=0.001
    if plane=="G17":
        if first.z != second.z or second.z != third.z:
            return 2
        else:
            temp = cl.position(2*second.x-first.x,2*second.y-first.y,first.z,0,0,0,0)
            l1=LinearLength(second,temp)
            l2=LinearLength(second,third)
            l3=LinearLength(temp,third)
            cos=(l1**2+l2**2-l3**2)/(2*l1*l2)
            if np.abs(cos-1)<precision:
                return 1
            else:
                return cos
    elif plane=="G18":
        if first.y != second.y or second.y != third.y:
            return 2
        else:
            temp=cl.position(2*second.x-first.x, first.y,2*second.z-first.z,0,0,0,0)
            l1=LinearLength(second,temp)
            l2=LinearLength(second,third)
            l3=LinearLength(temp,third)
            cos=(l1**2+l2**2-l3**2)/(2*l1*l2)
            if np.abs(cos-1)<precision:
                return 1
            else:
                return cos
    else:
        if first.x != second.x or second.x !=third.x:
            return 2
        else:
            temp=cl.position(first.x, 2*second.y-first.y, 2*second.z-first.z,0,0,0,0)
            l1=LinearLength(second,temp)
            l2=LinearLength(second,third)
            l3=LinearLength(temp,third)
            cos=(l1**2+l2**2-l3**2)/(2*l1*l2)
            if np.abs(cos-1)<precision:
                return 1
            else:
                return cos
            

def LinearLength(first,second):
    value = np.sqrt((first.x-second.x)**2+(first.y-second.y)**2+(first.z-second.z)**2)
    return value

def CircularLength(first,second,center,motiontype,plane):
    length=0
    a1=0
    a2=0
    b1=0
    b2=0

    if plane=="G17":
        a1=first.x-center.x
        a2=first.y-center.y
        b1=second.x-center.x
        b2=second.y-center.y
        cos=1-((second.x-first.x)**2+(second.y-first.y)**2)/(2*center.r**2)
    elif plane=="G18":
        a1=first.x-center.x
        a2=first.z-center.z
        b1=second.x-center.x
        b2=second.z-center.z
        cos=1-((second.x-first.x)**2+(second.z-first.z)**2)/(2*center.r**2)
    else:
        a1=first.y-center.y
        a2=first.z-center.z
        b1=second.y-center.y
        b2=second.z-center.z
        cos=1-((second.y-first.y)**2+(second.z-first.z)**2)/(2*center.r**2)
    
    value=a1*b2-b1*a2
    angle=np.arccos(cos)
    if value>0:
        if motiontype=="G02" or motiontype=="G2":
            length=(2*np.pi-angle)*center.r
        else:
            length=angle*center.r
    elif value==0:
        length=np.pi*center.r
    else:
        if motiontype =="G02" or motiontype=="G2":
            length=angle*center.r
        else:
            length=(2*np.pi-angle)*center.r

    return length

def TangentPoint(first,center,length, motiontype,plane):
    if plane=="G17":
        if motiontype=="G02" or motiontype=="G2":
            cos_alpha=(first.y-center.y)/center.r
            cos_beta=(center.x-first.x)/center.r
        else:
            cos_alpha=(center.y-first.y)/center.r
            cos_beta=(first.x-center.x)/center.r
        endpoint=cl.position(first.x+length*cos_alpha,first.y+length*cos_beta,first.z,0,0,0,0)
    elif plane=="G18":
        if motiontype=="G02" or motiontype=="G2":
            cos_alpha=(first.z-center.z)/center.r
            cos_beta=(center.x-first.x)/center.r
        else:
            cos_alpha=(center.z-first.z)/center.r
            cos_beta=(first.x-center.x)/center.r
        endpoint=cl.position(first.x+length*cos_alpha, first.y, first.z+length*cos_beta,0,0,0,0)
    else:
        if motiontype=="G02" or motiontype=="G2":
            cos_alpha=(first.z-center.z)/center.r
            cos_beta=(center.y-first.y)/center.r
        else:
            cos_alpha=(center.z-first.z)/center.r
            cos_beta=(first.y-center.y)/center.r
        endpoint=cl.position(first.x,first.y+length*cos_alpha,first.z+length*cos_beta,0,0,0,0)
    return endpoint

# About solving a cubic equation:https://www.cnblogs.com/larissa-0464/p/11706131.html
def CubicEquation(args):
    a,b,c,d=args
    p=c/a-b**2/(3*a**2)
    q=d/a+2*b**3/(27*a**3)-b*c/(3*a**2)
    w=complex(-0.5,(3**0.5)/2)
    ww=complex(-0.5,-(3**0.5)/2)
    A=cmath.sqrt((q/2)**2+(p/3)**3)
    B=ThreeSquare(-q/2+A)
    C=ThreeSquare(-q/2-A)
    y1=B+C
    y2=w*B+ww*C
    y3=ww*B+w*C
    D=b/(3*a)
    return y1-D,y2-D,y3-D

def ThreeSquare(x):
    if x.imag==0:
        m=x.real
        ans=-((-m)**(1/3)) if m<0 else m**(1/3)
    else:
        ans=x**(1/3)
    return ans

def QuarticEquation(args):
    a,b,c,d,e=args
    P=(c**2+12*a*e-3*b*d)/9
    Q=(27*a*d**2+2*c**3+27*b**2*e-72*a*c*e-9*b*c*d)/54
    D=cmath.sqrt(Q**2-P**3)
    u=ThreeSquare(Q+D) if abs(Q+D)>=abs(Q-D) else ThreeSquare(Q-D)
    v=0 if u==0 else P/u
    w=complex(-0.5,3**0.5/2)
    m=[]
    M=[]
    flag=0
    roots=[]
    for i in range(3):
        x=cmath.sqrt(b**2-8*a*c/3+4*a*(w**i*u+w**(3-i)*v))
        m.append(x)
        M.append(abs(x))
        if m==0:
            flag=flag+1
    if flag==3:
        mm=0
        S=b**2-8*a*c/3
        T=0
    else:
        t=M.index(max(M))
        mm=m[t]
        S=2*b**2-16*a*c/3-4*a*(w**t*u+w**(3-t)*v)
        T=(8*a*b*c-16*a**2*d-2*b**3)/mm
    x1=(-b-mm+cmath.sqrt(S-T))/(4*a)
    x2=(-b-mm-cmath.sqrt(S-T))/(4*a)
    x3=(-b+mm+cmath.sqrt(S+T))/(4*a)
    x4=(-b+mm-cmath.sqrt(S+T))/(4*a)
    roots.append(x1)
    roots.append(x2)
    roots.append(x3)
    roots.append(x4)
    return roots

def BinaryEquation(args):
    a,b,c,m,n,z=args
    x=(c*n-b*z)/(a*n-b*m)
    y=(c*m-a*z)/(b*m-a*n)
    return x,y