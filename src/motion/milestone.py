class Milestone():
    def __init__(self, t, q, vacuum_status, simulation_status):
        self.t = t
        self.robot = q
        self.gripper = [0,0,0]
        self.vacuum = [vacuum_status]
        self.simulation = simulation_status

    def get_milestone(self):
        return (self.t, {
                  'self.robot': self.robot,
                  'gripper': self.gripper,
                  'vacuum': [self.vacuum],
                  'simulation': self.simulation
                })

    #def set_milestone(self, milestone):
    #    self.milestone = milestone

    def get_t(self):
        return self.t

    def set_t(self, t):
        self.t = t

    def get_robot(self):
        return self.robot

    def set_robot(self, robot):
        self.robot = robot

    def get_gripper(self):
        return self.gripper

    def set_gripper(self, gripper):
        self.gripper = gripper

    def get_vacuum(self):
        return self.vacuum

    def set_vacuum(self, vacuum):
        self.vacuum = vacuum

    def get_simulation(self):
        return self.simulation

    def set_simulation(self, simulation):
        self.simulation = simulation


    #Fixes an array of milestones
    def fix_milestones(motion_milestones):
        max_change=5.0/180.0*3.14159
        max_speed=60/180.0*3.14159

        old_config=motion_milestones[0][1]['self.robot']
        i=1
        while i<len(motion_milestones):
            new_config=motion_milestones[i][1]['self.robot']
            d_config=max(max(vectorops.sub(new_config,old_config)),-min(vectorops.sub(new_config,old_config)))
            speed_config=d_config/motion_milestones[i][0]
            if d_config>max_change:
                new_milestone=Milestone(motion_milestones[i][0],vectorops.div(vectorops.add(motion_milestones[i-1][1]['self.robot'],motion_milestones[i][1]['self.robot']),2),motion_milestones[i-1][1]['vacuum'][0],motion_milestones[i-1][1]['simulation']).get_milestone()
                motion_milestones.insert(i,new_milestone)
                continue
            elif speed_config>max_speed:
                new_milestone=Milestone(d_config/(milestone_check_max_speed-0.1),motion_milestones[i][1]['self.robot'],motion_milestones[i][1]['vacuum'][0],motion_milestones[i][1]['simulation']).get_milestone()
                motion_milestones[i]=new_milestone
                i+=1
                old_config=new_config
            else:
                i+=1
                old_config=new_config
        return motion_milestones
