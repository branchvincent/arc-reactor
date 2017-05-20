from master.fsm import State

class FindAll(State):
    def run(self):
        self.setOutcome(True)

        #look at camera view and ID any and all items in view


        #take camera pic
        #ID shelf, location, etc
        #get point cloud

        #self.store.put('/item/'+self.ItemDict['name']+'/location',  self.foundLoc)



    
if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'fa')
    FindAll(myname).run()

