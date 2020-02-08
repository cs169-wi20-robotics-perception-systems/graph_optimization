import numpy as np
import g2o

class PoseGraphOptmization(g2o.SparseOptimizer):
    def __init__(self):
        super(PoseGraphOptmization, self).__init__()
        solver = g2o.BlockSolverSE2(g2o.LinearSolverCholmodSE2())
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        super(PoseGraphOptmization, self).set_algorithm(solver)


    def optimize(self, max_iterations=20):
        """
        Function for optimizing the 2D graph.

        Args:
            max_iterations: Maximum number of iterations in case of no convergence.
        """
        super(PoseGraphOptmization, self).initialize_optimization()
        super(PoseGraphOptmization, self).optimize(max_iterations)


    def add_vertex(self, id, pose, fixed=False):
        """
        Adds 2D vertex to the graph.

        Args:
            id: Id number of the to-be created vertex.
            pose: Numpy array of the pose of the to-be created vertex.
            fixed: Unknown :)
        """
        v_se2 = g2o.VertexSE2()
        v_pose = g2o.SE2(pose)
        v_se2.set_id(id)
        v_se2.set_estimate(v_pose)
        v_se2.set_fixed(fixed)
        super(PoseGraphOptmization, self).add_vertex(v_se2)


    def add_edge(self, vertices, measurement, information=np.identity(3), robust_kernel=None):
        """
        Adds 2D edge to the graph.

        Args:
            vertices: Two ids of the vertices that will make the edge.
            measurement: Numpy array of that describes the constraint between the two verticesself.
            information: The information matrix that describes the belief of the sensor measurements.
            robust_kernel: Unknown :)
        """
        edge = g2o.EdgeSE2()
        for i, v in enumerate(vertices):
            if isinstance(v, int):
                v = self.vertex(v)
            edge.set_vertex(i, v)

        e_measurement = g2o.SE2(measurement)
        edge.set_measurement(e_measurement)
        edge.set_information(information)
        if robust_kernel is not None:
            edge.set_robust_kernel(robust_kernel)
        super(PoseGraphOptmization, self).add_edge(edge)


    def get_pose(self, id):
        """
        Returns the vertex object with the given id.

        Args:
            id: The id of the wanted vertex.

        Return:
            The object of the wanted vertex.
        """
        return self.vertex(id).estimate()
