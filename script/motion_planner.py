"""
TP Motion Planning
3A SRI 2020
Maxence Da Silva - Clément Truillet

"""

class MotionPlanner:
  def __init__ (self, robot, ps):
    self.robot = robot
    self.ps = ps

  def solveBiRRT (self, maxIter = float("inf")):
    self.ps.prepareSolveStepByStep()
    finished = False

    # In the framework of the course, we restrict ourselves to 2 connected components.
    nbCC = self.ps.numberConnectedComponents()
    if nbCC != 2:
      raise Exception("There should be 2 connected components.")

    iter = 0
    while True:
    
      #### RRT begin
      newConfigs = list()
      # newConfigs.append(ps.getInitialConfig())
      # newConfigs.add(ps.getGoalConfigs())
			
      for i in range(nbCC):
				# Création d'une configuration random du robot
        qRandom = self.robot.shootRandomConfig()
				
				# Création de la configuration la plus proche
        qNear, dNear = self.ps.getNearestConfig(qRandom)
				
				# Crée le chemin entre la configuration la plus proche et la configuration random
        validation, pathId, _ = self.ps.directPath(qNear, qRandom, True)
        qNew = self.ps.configAtParam(pathId, self.ps.pathLength(pathId))
				
				# Ajoute le chemin à l'arbre
        self.ps.addConfigToRoadmap(qNew)
        self.ps.addEdgeToRoadmap(qNear, qNew, pathId, True)
        newConfigs.append(qNew)

      ## Try connecting the new nodes together
      for i in range(len(newConfigs)):
        if newConfigs[i] == qNew:
          pass
        else:
          validation, pathId, _ = self.ps.directPath(newConfigs[i], qNew, True)
          if validation:
            self.ps.addEdgeToRoadmap(newConfigs[i], qNew, pathId, True)
        
      #### RRT end
      
      ## Check if the problem is solved.
      nbCC = self.ps.numberConnectedComponents()
      if nbCC == 1:
        # Problem solved
        finished = True
        break
      iter = iter + 1
      if iter > maxIter:
        break
    if finished:
        self.ps.finishSolveStepByStep()
        return self.ps.numberPaths() - 1

  def solvePRM (self):
    self.ps.prepareSolveStepByStep()
    #### PRM begin
    #### PRM end
    self.ps.finishSolveStepByStep()
