class position:
    def __init__(self, x, y,z,i,j,k,r):
        self.x=x
        self.y=y
        self.z=z
        self.i=i
        self.j=j
        self.k=k
        self.r=r
    
class motion:
    def __init__(self,motion_type,position,order,feedrate,plane):
        self.motion_type=motion_type
        self.position=position
        self.order=order
        self.feedrate=feedrate
        self.plane=plane

