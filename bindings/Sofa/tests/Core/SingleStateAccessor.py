# coding: utf8

import unittest
import Sofa
import Sofa.Core
import Sofa.Helper
import Sofa.Simulation
import SofaRuntime

class Test(unittest.TestCase):

    @staticmethod
    def simulate_beam():
        root = Sofa.Core.Node("rootNode")

        root.addObject('DefaultAnimationLoop')

        root.addObject('RequiredPlugin', name='Sofa.Component.ODESolver.Backward')
        root.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Direct')
        root.addObject('RequiredPlugin', name='Sofa.Component.Engine.Select')
        root.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Projective')
        root.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.FEM.Elastic')
        root.addObject('RequiredPlugin', name='Sofa.Component.Mass')

        root.addObject('EulerImplicitSolver', rayleighStiffness="0.1", rayleighMass="0.1")
        root.addObject('SparseLDLSolver', applyPermutation="false", template="CompressedRowSparseMatrixd")

        root.addObject('MechanicalObject', name="DoFs")
        root.addObject('MeshMatrixMass', name="mass", totalMass="320")
        root.addObject('RegularGridTopology', name="grid", nx="4", ny="4", nz="20", xmin="-9", xmax="-6", ymin="0", ymax="3", zmin="0", zmax="19")
        root.addObject('BoxROI', name="box", box=[-10, -1, -0.0001,  -5, 4, 0.0001])
        root.addObject('FixedConstraint', name="fixed_constraint", indices="@box.indices")
        root.addObject('HexahedronFEMForceField', name="FEM", youngModulus="4000", poissonRatio="0.3", method="large")

        Sofa.Simulation.init(root)

        return root

    def test_single_state_accessor_get_mstate(self):

        root = self.simulate_beam()

        mass_mstate = root.mass.get_mstate()
        self.assertNotEqual(mass_mstate, None)

        fem_mstate = root.FEM.get_mstate()
        self.assertNotEqual(fem_mstate, None)

        self.assertEqual(fem_mstate, mass_mstate)

        self.assertEqual(fem_mstate, root.DoFs)

