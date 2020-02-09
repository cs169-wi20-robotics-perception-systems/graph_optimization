import g2o

def write_optimized_graph(g2o_graph_optimization, num_vertices, file_name):
    """
    Writes the optimized graph to a new g2o file.

    Args:
        g2o_graph_optimization: The optimized graph.
        num_vertices: The number of vertices in the graph.
        file_name: The path and file name that will be written to.
    """
    f = open(file_name, "w")
    for i in range(num_vertices + 1):
        f.write(format_vertex(g2o_graph_optimization.get_pose(i), i) + "\n")
    f.close()


def append_vertex(vertex, id, file_name):
    """
    Appends the written description of a new vertex.

    Args:
        vertex: The vertex to be written about.
        id: The id of the vertex.
        file_name: The path and file name that will be written to.
    """
    f = open(file_name, "a")
    f.write(format_vertex(vertex, id) + "\n")
    f.close()


def append_edge(id_i, id_j, change_in_state, information_matrix, file_name):
    """
    Appends the written description of a new edge.

    Args:
        id_i: The id of the first vertex.
        id_j: The id of the second vertex.
        change_in_state: The change in state between the two vertices.
        information_matrix: The information matrix described by the confidence of the sensors.
        file_name: The path and file name that will be written to.
    """
    f = open(file_name, "a")
    f.write(format_edge(id_i, id_j, change_in_state, information_matrix) + "\n")
    f.close()


def format_vertex(vertex, id):
    """
    Creates the written description of the vertex.

    Args:
        vertex: The vertex object.
        id: The id of the vertex.

    Returns:
        The string that describes the given vertex.
    """
    return "VERTEX_SE2 " + (str(id) +
                        " " + str(vertex[0]) +
                        " " + str(vertex[1]) +
                        " " + str(vertex[2]))

def format_edge(id_i, id_j, change, information_matrix):
    """
    Creates the written description of the edge.

    Args:
        id_i: The first vertex id.
        id_j: The second vertex id.
        change: The change in state between the two vertices.
        information_matrix: The information matrix described by the confidence of the sensors.

    Returns:
        The string that describes the given edge.
    """
    output_string = "EDGE_SE2 " + (str(id_i) +
                                " " + str(id_j) +
                                " " + str(change[0]) +
                                " " + str(change[1]) +
                                " " + str(change[2]))

    output_string = output_string + (" " + str(information_matrix[0][0]) +
                                    " " + str(information_matrix[0][1]) +
                                    " " + str(information_matrix[0][2]) +
                                    " " + str(information_matrix[1][1]) +
                                    " " + str(information_matrix[1][2]) +
                                    " " + str(information_matrix[2][2]))
    return output_string
