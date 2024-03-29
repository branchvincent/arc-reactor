import socket
import time
import functools

class TrajClient:
    """ An implementation of the VAL3 trajectory client API to be used
    with the trajServer program.

    See the accompanying documentation for information about the
    commands and motion queue semantics.
    """
    
    VERSION = "1.0"
    
    def __init__(self,host=None,port=1000):
        self.s = None
        self.name = "Unnamed"
        self.curId = 0
        self.flog = None
        self.partialInput = ""
        if host != None:
            self.connect(host,port)

    def logBegin(self,fn='trajclient.log'):
        self.flog = open(fn,"a")

    def logEnd(self):
        if self.flog != None:
            self.flog.close()
            self.flog = None
    
    def connect(self,host,port=1000):
        if self.name == "Unnamed":
            self.name = host+":"+str(port)
        self.s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.s.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
        res = self.s.connect((host,port))
        if self.version()!=self.VERSION:
            raise ValueError("Connected to version "+self.version()+" of trajClient, need "+self.VERSION)
        self.partialInput = ""

    def sendMessageRaw(self,cmds,wantReply=True):
        """Sends the commands cmds, which may be one or more function calls.
        Does not wait for a return value from the server.
        The method will return the ID of the expected return message, or -1
        if no message will be returned.

        Warning: if wantReply is true (it is by default), then receiveReply()
        must be called before sending additional messages!
        """
        retval = -1
        if wantReply==False:
            msg = '* '
        else:
            retval = self.curId
            msg = str(self.curId)+' '
        if hasattr(cmds,'__iter__'):
            msg = msg+','.join(cmds)+';'
        else:
            msg = msg+cmds+';'
            
        # Send the message
        if self.flog != None:
            self.flog.write(time.asctime()+": "+self.name+" <- "+msg+"\n")
        while len(msg) > 0:
            n = self.s.send(msg)
            if n < 0:
                raise RuntimeError("Send returned < 0, connection closed")
            msg = msg[n:]
        self.curId = self.curId+1
        return retval

    def receiveReply(self,delay=0):
        """Returns the entire reply message without parsing.
        Warning: this must only be called when awaiting a reply to an earlier
        message.  Otherwise, it will block and never return.
        """
        done = False
        if not hasattr(self,'lasttime'):
            self.lasttime = time.time()
        while not done:
            splits = self.partialInput.split(';',1)
            if len(splits) > 1:
                msg = splits[0]
                self.partialInput = splits[1]
                if self.flog != None:
                    self.flog.write(time.asctime()+": "+self.name+" -> "+msg+"\n")
                return msg
            msg = self.s.recv(1460)
            #filter out null characters
            msg = filter(lambda(c):c!='\0',msg)
            t = time.time()
            #print t-self.lasttime,len(msg),'"'+msg+'"'
            self.lasttime = t
            self.partialInput += msg
            if len(msg) == 0:
                #does this indicate disconnection?
                if delay > 0 and not done:
                    time.sleep(delay)



    def sendMessage(self,cmds,wantReply=True):
        """Sends the command strings cmds, which may be one or more function calls.
        Returns the list of return values.
        """
        msgid = self.sendMessageRaw(cmds,wantReply)
        
        # Receive the reply
        if wantReply:
            ret = self.receiveReply()
            (retid,s) = ret.strip().split(" ",1)
            res = s.rstrip(";").split(",")
            if "Invalid" in res:
                raise ValueError("Invalid command: "+cmds[res.index("Invalid")])
            if not hasattr(cmds,'__iter__'):
                return res[0]
            else:
                return res
        else:
            return None

    def call(self,calls,wantReply=True):
        """Makes the given function calls and returns a list of return values.
        A function call is either the call name 'func', or a tuple
        (func,arg1,...,argn)
        
        Can be called using a list of calls or just a single call.
        In the latter case, just returns the return value.
        """
        def callToMsg(call):
            if type(call) is str:
                return call+"()"
            return call[0]+"("+" ".join(str(a) for a in call[1:])+")"

        if type(calls) is str or type(calls) is tuple:
            return self.sendMessage(callToMsg(calls),wantReply)
        else:
            return self.sendMessage([callToMsg(c) for c in calls],wantReply)

    def echo(self,message):
        return self.call(("echo",message))

    def version(self):
        return self.call("version")

    def rate(self):
        return float(self.call("rate"))

    def getConfig(self):
        return [float(x) for x in self.call("gj").split()]
    
    def getVelocity(self):
        return [float(x) for x in self.call("gv").split()]

    def getTransform(self):
        return [float(x) for x in self.call("gx").split()]

    def getJointLimits(self):
        return ([float(x) for x in self.call("gjmin").split()],[float(x) for x in self.call("gjmax").split()])

    def getVelocityLimits(self):
        return [float(x) for x in self.call("gvl").split()]

    def getAccelerationLimits(self):
        return [float(x) for x in self.call("gal").split()]

    def getDecelerationLimits(self):
        return [float(x) for x in self.call("gdl").split()]

    def setJointLimits(self,qmin,qmax):
        if len(qmin)!=6 or len(qmax)!=6:
            raise ValueError("Joint limit vectors must have 6 elements")
        self.call(("sjmin",)+tuple(qmin))
        self.call(("sjmax",)+tuple(qmax))

    def setVelocityLimits(self,vmax):
        if len(vmax)!=6:
            raise ValueError("Velocity limit vector must have 6 elements")
        self.call(("svl",)+tuple(vmax));

    def setAccelerationLimits(self,amax):
        if len(amax)!=6:
            raise ValueError("Acceleration limit vector must have 6 elements")
        self.call(("sal",)+tuple(amax));

    def setDecelerationLimits(self,amax):
        if len(amax)!=6:
            raise ValueError("Acceleration limit vector must have 6 elements")
        self.call(("sdl",)+tuple(amax));

    def getMaxSegments(self):
        return int(self.call("gms"))

    def getCurSegments(self):
        return int(self.call("gcs"))
    
    def getRemainingSegments(self):
        res = self.call(["gms","gcs"])
        return int(res[0])-int(res[1])

    def getCurrentTime(self):
        return float(self.call("gct"))

    def getTrajEndTime(self):
        return float(self.call("get"))

    def getTrajDuration(self):
        return float(self.call("gd"))

    def getEndConfig(self):
        return [float(x) for x in self.call("gej").split()]
    
    def getEndVelocity(self):
        return [float(x) for x in self.call("gev").split()]

    def addMilestone(self,dt,q):
        """Appends the milestone q to be reached at the time dt after the
        end of the motion queue (or the time the command is received, if
        the motion queue is currently empty).  The robot will perform
        a linear interpolation in joint space from the prior configuration
        to q."""
        res=self.call(("am",dt)+tuple(q))
        if res=="Error":
            raise ValueError("addMilestone failed")
        else:
            #returns milestone index
            return int(res)

    def addMilestoneQuiet(self,dt,q):
        return self.call(("am",dt)+tuple(q),wantReply=False)

    def appendMilestones(self,dts,qs):
        """Appends milestones in q to the end of the trajectory queue,
           separated respectively by the times in dts.  Returns the
           indices of the resulting segments."""
        if len(dts) != len(qs):
            raise ValueError("Invalid argument lengths")
        if len(dts) == 0: return []
        calls = [()]*len(dts)
        for i in xrange(len(dts)):
            calls[i] = ("am",dts[i])+tuple(qs[i])
        res=self.call(calls)
        if "Error" in res:
            raise ValueError("appendMilestones failed")
        else:
            #returns milestone index
            return [int(r) for r in res]

    def appendMilestonesQuiet(self,dts,qs):
        """Same as appendMilestones, but with no return value"""
        if len(dts) != len(qs):
            raise ValueError("Invalid argument lengths")
        if len(dts) == 0: return
        calls = [()]*len(dts)
        for i in xrange(len(dts)):
            calls[i] = ("am",dts[i])+tuple(qs[i])
        self.call(calls,wantReply=False)

    def resetTrajectoryAbs(self,t):
        """Resets the trajectory at absolute (controller) time t."""
        res=self.call(("rtabs",t))
        if res=="Error":
            raise ValueError("resetTrajectoryAbs failed")
        else:
            return None

    def resetTrajectoryRel(self,dt):
        """Resets the trajectory at time dt after the instant that the call is
        received."""
        res=self.call(("rtrel",dt))
        if res=="Error":
            raise ValueError("resetTrajectoryRel failed")
        else:
            return None

    def checkTrajectory(self):
        """Performs a trajectory check on the controller"""
        res=self.call("check")
        if res=="":
            return None
        else:
            raise ValueError("Trajectory infeasible: "+res)




class ReadOnlyTrajClient:
    """A 'sanitized' version of the TrajClient.  No movement-generating
    commands are allowed in order to help prevent accidents."""
    def __init__(self,trajClient):
        self._trajClient = trajClient

def calltrajclientfunc(_self,name,*args):
    print "name=",name
    print "self=",_self
    print "args=",args
    return getattr(_self._trajClient,name)(*args)

def partialfakemethod(fakemethod, *args, **kw):
    def call(obj, *more_args, **more_kw):
        #print "obj=",obj
        #print "args=",more_args
        call_kw = kw.copy()
        call_kw.update(more_kw)
        return fakemethod(obj,*(args+more_args), **call_kw)
    return call

safeFuncs = ["checkTrajectory","echo","version","rate",
             "getConfig","getVelocity","getTransform",
             "getJointLimits","getVelocityLimits",
             "getAccelerationLimits","getDecelerationLimits",
             "getMaxSegments","getCurSegments","getRemainingSegments",
             "getCurrentTime","getTrajEndTime","getTrajDuration",
             "getEndConfig","getEndVelocity"]
        
for f in safeFuncs:
    setattr(ReadOnlyTrajClient,f,partialfakemethod(calltrajclientfunc,f))

