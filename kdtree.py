from Scientific.Visualization.VRML2 import *
from numpy.oldnumeric import array
from sets import Set
from numpy.oldnumeric.random_array import uniform,permutation,randint
from os import system
class AABB:
    """Describes an axis-aligned box """
    def __init__(self,up,down):
        for i in range(3):
            if up[i]<down[i]:
                up[i],down[i]=down[i],up[i]
        self.up=up
        self.down=down
        self.center=(up+down)/2.
        self.height=self.up-self.down
        
    def splitN(self,n):
        """ splits the AABB along the N'th axis"""
        mid=(self.up[n]+self.down[n])/2
        midu=[self.up[0],self.up[1],self.up[2]]
        midu[n]=mid
        midd=[self.down[0],self.down[1],self.down[2]]
        midd[n]=mid
        midu=Vector(midu)
        midd=Vector(midd)
        ##print str(self)+'/2'+['x','y','z'][n]+' midu='+str(midu)
        a=AABB(self.up,midd),AABB(midu,self.down)
        ##print str(a[0]),str(a[1])
        return a

    def splitX(self):
        """ Splits the AABB x-wise"""
        return self.splitN(0)

    def splitY(self):
        """ Splits the AABB y-wise"""
        return self.splitN(1)

    def splitZ(self):
        """ Splits the AABB z-wise"""
        return self.splitN(2)

    def splitXYZ(self):
        """Splits the AABB equally in all three directions """
        return [[list(j[0].splitZ()),list(j[1].splitZ())]
                for j in [list(i.splitY()) for i in self.splitX()]]
        
    def __str__(self):
        return '{'+str(self.up)+'-'+str(self.down)+'}'

    def vis(self,lines=False):
        """ Returns VRML visualizstion of the AABB"""
        if lines:
            return [PolyLines([Vector(self.up.x(),self.up.y(),self.up.z()),
                               Vector(self.up.x(),self.up.y(),self.down.z()),
                               Vector(self.up.x(),self.down.y(),self.down.z()),
                               Vector(self.up.x(),self.down.y(),self.up.z()),
                               Vector(self.down.x(),self.down.y(),self.up.z()),
                               Vector(self.down.x(),self.down.y(),self.down.z()),
                               Vector(self.down.x(),self.up.y(),self.down.z()),
                               Vector(self.down.x(),self.up.y(),self.up.z()),
                               Vector(self.up.x(),self.up.y(),self.up.z())])]
        else:
            return [Polygons([Vector(self.up.x(),self.up.y(),self.up.z()),
                              Vector(self.up.x(),self.up.y(),self.down.z()),
                              Vector(self.up.x(),self.down.y(),self.down.z()),
                              Vector(self.up.x(),self.down.y(),self.up.z()),
                              Vector(self.down.x(),self.down.y(),self.up.z()),
                              Vector(self.down.x(),self.down.y(),self.down.z()),
                              Vector(self.down.x(),self.up.y(),self.down.z()),
                              Vector(self.down.x(),self.up.y(),self.up.z())],
                             [[3,2,1,0],[7,6,5,4],[2,3,4,5],[6,7,0,1],
                              [1,2,5,6],[7,4,3,0]],
                             material=Material(transparency=0.3))]
##[0,1,2,3],[4,5,6,7]
    def __gt__(self, other):
        """ Is the other AABB completely enclosed in self """
        return sum([self.up[i]>other.up[i] and
                    self.down[i]<other.down[i] for i in range(3)])==3

    def __lt__(self,other):
        """ Does the other AABB completely enclose self?"""
        return sum([other.up[i]>self.up[i] and
                    other.down[i]<self.down[i] for i in range(3)])==3

    def __contains__(self,other):
        """ Is the vector other inside self?"""
        return sum([other[i]<self.up[i] and
                    other[i]>self.down[i] for i in range(3)])==3

    def uniformWithin(self):
        """Returns a random point inside self"""
        return Vector(uniform(0.,1.,(3,))*self.height.array+self.down.array)

    def makeTests(self, scale):
        """ Creates a scaled random AABB inside self"""
        p=permutation(3)    
        size=self.height.array*uniform(0.5,1.5,(3,))*scale
        size=(size[p[0]],size[p[1]],size[p[2]])
        where=self.uniformWithin()
        return AABB(where+Vector(size),where)

class Octtree:
    """A special purpose octttree class for configuration space use """
    def __init__(self,aabb,limit):
        self.limit=limit
        #if limit!=0:
         #   self.children=[[[Octtree(k,limit-1) for k in j]
          #                  for j in i] for i in aabb.splitXYZ()]
        #else:
        self.children=None
        self.aabb=aabb
        self.leaves=Set()

    def insert(self, object):
        """ Inserts an object into the Octtree"""
        if self.aabb>object.aabb:##check the sum
            #print str(self.aabb)+">"+str(object.aabb)
            if self.children==None:
                if self.limit==0:
                    self.leaves.add(object)
                    return True
                self.children=[[[Octtree(k,self.limit-1) for k in j]
                                for j in i] for i in self.aabb.splitXYZ()]
                #else:
            if not sum([sum([sum([k.insert(object) for k in j])
                             for j in i]) for i in self.children]):
                self.leaves.add(object)
                return True
        #print  "NOT-"+str(self.aabb)+">"+str(object.aabb)
        return False
    

    def vis(self,top=False):
        """Returns VRML visualization of the extent of the configuration space and
        all obstacles contained within it"""
        if top:
            c=self.aabb.vis(True)
        else:
            c=[]
        if len(self.leaves)>0:
            for i in self.leaves:
                c=c+i.vis()
        if self.children!=None:
            for i in self.children:
                for j in i:
                    for k in j:
                        c=c+k.vis()
        return c
            

    def __str__(self):
        top=",".join(map(str,self.leaves))
        if self.children is not None:
            for i in self.children:
                for j in i:
                    for k in j:
                        top=top+str(k)
        return top

    def __contains__(self,v):
        """ Returns true if the point is inside the configuration space
        and not inside an obstacle"""
        if v in self.aabb:
            #print v, "in ", self.aabb
            for i in self.leaves:
                if v in i.aabb:
                    #print v, "in leaf", i.aabb 
                    return False
            if self.children is not None:
                if v[0] > self.aabb.center[0]:
                    if v[1] > self.aabb.center[1]:
                        if v[2] > self.aabb.center[2]:
                            return v in self.children[0][0][0]
                        else:
                            return v in self.children[0][0][1]
                    else:
                        if v[2] > self.aabb.center[2]:
                            return v in self.children[0][1][0]
                        else:
                            return v in self.children[0][1][1]
                else:
                    if v[1] > self.aabb.center[1]:
                        if v[2] > self.aabb.center[2]:
                            return v in self.children[1][0][0]
                        else:
                            return v in self.children[1][0][1]
                    else:
                        if v[2] > self.aabb.center[2]:
                            return v in self.children[1][1][0]
                        else:
                            return v in self.children[1][1][1]
            else:
                return True
        else:
            #print v, "not in", self.aabb 
            return False

    def makeTests(self,amount,scale):
        """Fills the configuaration space with a number of obstacles """
        for i in range(amount):
            self.insert(Geo(self.aabb.makeTests(scale)))

    def uniformWithin(self):
        """Returns points uniformly distributed inside the configuration space"""
        while True:
            v=self.aabb.uniformWithin()
            if v in self:
                return v
            
class Geo:
    """Geometry inside the configuration space- if there were more
    intersection tests, this is where they'd be"""
    def __init__(self,aabb):
        self.aabb=aabb

    def vis(self):
        """VRML Visualization of geometry"""
        return self.aabb.vis()

    def __str__(self):
        return str(self.aabb)

class SPoint:
    from math import acos
    def __init__(self,vec,dir=None):
        if dir is None:
            dir=Vector(0.,0.,0.)
        self.vec=vec
        self.dir=dir
    
    def __getitem__(self,which):
        #print which
        if which<3:
            return self.vec[which]
        else:
            return self.dir[which-3]

    def dist(self,sp):
        dir=(sp.vec-self.vec).normal()
        return (self.vec-sp.vec).length()+self.acos((self.dir*dir))*E

    def nextelement(self,sp):
        reqp=self.vec-(self.vec-sp.vec).normal()*2*E
        """ Create next element from 6-point"""
        pd=(reqp-self.vec).normal()
        np=self.vec+self.dir*2*E ##new point
        tmp=(reqp-np).normal()
        if pd==tmp:
            return self.nextelement(reqp+Vector(0.1,0.1,0.1))
        plane=self.dir.cross(pd).normal()
        ndir=tmp.cross(plane).normal()
        return SPoint(self.vec+ndir*2*E,ndir)

    def __str__(self):
        return "{"+str(self.vec)+",direction="+str(self.dir)+"}"
        
    def vis(self):
        return Arrow(self.vec,self.vec+(self.dir*E),E/10,material=Material(diffuse_color = Color((1.,0.,0.))))
    
class KDTree:
    def __init__(self,dim=3,index=0):
        self.dim = dim
        self.index = index
        self.split = None

    def addPoint(self,p):
        """This function adds another point to the KD-tree"""
        if self.split is None:
            self.split = p
            self.left = KDTree(self.dim, (self.index + 1) % self.dim)
            self.right = KDTree(self.dim, (self.index + 1) % self.dim)
        elif self.split[self.index] < p[self.index]:
            self.left.addPoint(p)
        else:
            self.right.addPoint(p)

    def nearestNeighbor(self,q,maxdist2):
        """Returns tuple (d,p) where p is the nearest neighbor and d is the
        distance to p. Distance must be within maxdist2; if it isn't
        function returns None.
        """
        solution = (maxdist2+1,None)
        if self.split is not None:
            solution = min(solution, (self.split.dist(q),self.split))
            d2split = (self.split[self.index] - q[self.index])**2
            if self.split[self.index] < p[self.index]:
                solution = min(solution,
                               self.left.nearestNeighbor(q,solution[0]))
                if d2split < solution[0]:
                    solution = min(solution,
                                   self.right.nearestNeighbor(q,solution[0]))
            else:
                solution = min(solution,
                               self.right.nearestNeighbor(q,solution[0]))
                if d2split < solution[0]:
                    solution = min(solution,
                                   self.left.nearestNeighbor(q,solution[0]))
        return solution

class PointMaker:
    from Scientific.Visualization.VRML2 import Vector
    from numpy.oldnumeric.random_array import uniform
    ##from random import randint
    def __init__(self,nos,part,E,space,begin=None,end=None,direction=None):
        """Pointmaker generates nos+1 points of which 1/part will be the the goal
        E is the stridelength and thus the accuracy, space is the
        configuration space in which points can be created"""
        if begin is None:
            begin=space.uniformWithin()
        if end is None:
            end=space.uniformWithin()
        if direction is None:
            direction=self.randomDirection()
        self.nos,self.part,self.begin,self.end=nos,part,begin,end
        self.space,self.E=space,E
        self.direction=direction
        
    def __call__(self):
        """This generator yields points within the configuration space, some
        random, some not"""
        space=self.space
        part=self.part
        yield True,SPoint(self.begin,self.direction) ##first point gets special treatment
        for i in range(self.nos):
            if randint(1,part)>1:
                #yield False,Vector(uniform(-5.,5.,(3,)))
                yield False,SPoint(space.uniformWithin())
            else:
                yield False,SPoint(self.end)

    def nearEnd(self,p):
        return (self.end-p.vec).length()<E

    def randomDirection(self):
        return Vector(uniform(0.5,1.5,(3,))*(randint(0,1,(3,))*2-1)).normal()


 
if __name__ == "__main__":        
    ##Point offset
    E=0.25
    ##A maximum of 350 points will be generated
    P=1000
    F=3
    configurationspace=Octtree(AABB(Vector(5.,5.,5.),Vector(-5.,-5.,-5.)),3)
    configurationspace.makeTests(7,(0.3,0.2,0.5))
    max_dist=configurationspace.aabb.height.length()
    V=Scene(configurationspace.vis(False))
    k = KDTree(6)
    finished=False
    points=PointMaker(P,F,E,configurationspace)
    for first,p in points(): ##range(n_points):
        #p = Vector(uniform(-5.,5.,(3,)))        
        if first:
            print 'Begin point',p
            k.addPoint(p)
        else:            
            d,q = k.nearestNeighbor(p,max_dist) 
            #p=q+((p-q).normal()*E)
            p=q.nextelement(p)
            if p in configurationspace:                
                p.parent=q
                if(points.nearEnd(p)):
                    finished=True
                    print "Reached Final Configuration", points.end
                    break
                V.addObject(Line(q.vec,p.vec))
                k.addPoint(p)

    if finished:
        def Solution(p):
            while(p.parent.vec!=points.begin):
                yield p.vec
                p=p.parent
            yield p.vec
        V.addObject(PolyLines(list(Solution(p)),
                              material=Material(diffuse_color = Color((1.,0.,0.)))))
    else:
        print "Did not reach final configuration in "+str(P)+" points."

    V.addObject(Cube(points.begin,E,material=Material(diffuse_color = Color((0.,1.,0.)))))
    V.addObject(Cube(points.end,E,material=Material(diffuse_color = Color((0.,0.,1.)))))
    V.writeToFile('/mnt/vis.wrl')
    
