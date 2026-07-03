import Sofa
import numpy as np

"""
RIGIDIFICATION is the process of telling a structural simulation that certain
sub-sections of an otherwise fully flexible body must maintain perfect, unbending
motion throughout the simulation. This simulates structures connected by bolts,
welds, or solid beams which prevent local deformation.

--- The Challenge ---
Standard Finite Element Analysis (FEA) assumes *all* nodes are independent
degrees of freedom (DoFs), meaning every point can stretch and bend freely.
When we force a section to be perfectly rigid, its movement is no longer
governed by local forces but by global rigid body kinematics (only 6 DoFs:
3 position, 3 orientation). We must combine these two fundamentally different
types of motion into one cohesive simulation system.

--- The Solution Strategy ---
We solve this by partitioning the geometry and using mapping techniques:
1. Partitioning: Divide the mesh nodes into two sets: a Deformable set and a Rigid set.
2. Separate Models: Run separate models for each set (DeformableDoFs & RigidDoFs).
3. Constraint/Mapping (The Key): We use the 'RigidMapping' to take the single,
   master rigid body motion (6 DoFs) and mathematically constrain *every* node in the
   rigidified subset to adopt that perfect rigid movement.
4. Combination: The final object uses 'SubsetMultiMapping' to assemble a complete state
   vector. It takes all nodes from both the deformable part and the constrained
   (but physically included) rigid nodes, allowing them to solve together as one composite body.

--- Where does the physics actually live? ---
It is easy to assume that forces/mass are attached to "deformableDoFs" and
"rigidifiedDoFs" directly, since those nodes are what visually move. This is
NOT the case here.

  * "deformableDoFs" only holds POSITIONS (a MechanicalObject) plus a boundary
    constraint (FixedProjectiveConstraint). It has no mass and no force field
    of its own, so on its own it would not do anything physically meaningful.
  * "rigidifiedDoFs" similarly only holds POSITIONS, and its motion is fully
    determined by the RigidMapping (it is a *mapped* / dependent DoF, not a
    free one that a solver integrates directly).

The actual physics (mass + the CorotationalFEMForceField that computes elastic
forces from the hexahedral mesh) is attached ONLY to the "composite" node,
downstream of "SubsetMultiMapping". SubsetMultiMapping is a two-way ("multi")
mapping: it not only pulls the positions of "deformableDoFs" and
"rigidifiedDoFs" together into "compositeDoFs" (position flow, parent -> child),
it also pushes the forces computed on "compositeDoFs" back down to its two
parents (force flow, child -> parent). Those forces then further propagate
through RigidMapping into the 6-DoF rigid body. So the composite node is the
single place where physics is defined, and the mappings are responsible for
distributing the resulting forces to the deformable nodes and to the rigid
body.

--- Why bother going through the composite node at all? (Linear system size) ---
Because "deformableDoFs" and "rigidifiedDoFs" hold no mass/force field, the
solver never needs to build or solve for them directly. The linear system
actually assembled and solved at each time step is much smaller than a fully
flexible simulation of the whole mesh would require: it only contains the
free deformable DoFs plus the 6 DoFs of the rigid body (instead of 3 DoFs for
every single node in the rigidified region). The FEM stiffness/mass matrices
are computed on the full mesh in "composite/physics", then PROJECTED down
onto this reduced system through the chain of mappings (SubsetMultiMapping,
then RigidMapping). If you inspect the assembled system matrix (e.g. via
"GlobalSystemMatrixImage"/SofaMatrix), you will see non-zero coupling terms
between the deformable DoFs and the rigid body's 6 DoFs -- this coupling is
exactly the trace, in the reduced linear system, of the FEM forces that were
projected through the mappings.

--- What will you actually see if you run this scene? ---
The visualization uses two colors to distinguish the two regions:
  * NAVY BLUE spheres: the free, fully flexible "deformableDoFs" nodes.
  * PINK spheres: the "rigidifiedDoFs" nodes, rigidly attached to the master
    rigid body (also drawn as a frame via showObject=True on "rigidDoF").
Under gravity, the bottom of the structure is pinned (FixedProjectiveConstraint),
so the whole beam-like object will sag/bend downward over time. The navy
sections (top and bottom, outside the box ROI) will visibly deform and curve,
while the pink section (the middle block selected by the box ROI) will move
and rotate as a single perfectly rigid chunk -- you should NOT see any
bending, stretching, or relative motion between the pink spheres themselves,
only the whole pink block translating/rotating together, connected to the
bending navy sections at its two ends.

--- Order of mapping operations matters ---
Forces are accumulated by traversing the scene graph from the leaves/bottom
(composite/physics, where forces are actually computed) back up toward the
root, i.e. bottom-to-top in the graph. Concretely, that means:
  1. Forces are first computed on "compositeDoFs" (mass + FEM force field).
  2. SubsetMultiMapping then propagates ("applies") those forces back down to
     its two parents: "deformableDoFs" and "rigidifiedDoFs".
  3. Only after that step does RigidMapping propagate the force received by
     "rigidifiedDoFs" further down to "rigidDoF", converting it into an
     equivalent force + torque on the rigid body.
If this order were reversed, or a mapping's contribution were applied before
all of the forces feeding into it were known, the accumulated force/torque on
"rigidDoF" (and consequently the rigid body's motion) would be wrong. SOFA's
mapping graph handles this ordering automatically as long as the parent/child
relationships declared above (who is "input" and who is "output" of each
mapping) correctly reflect this bottom-to-top dependency.
"""


def createScene(root_node):
    root_node.name = "root"
    root_node.dt = 0.02
    root_node.gravity = [0, -9.81, 0]
    root_node.bbox = [[-5, -5, -5], [5, 5, 25]]

    # Load the SOFA plugins that provide the components used below
    # (solvers, mappings, mass, force fields, visualization, etc.)
    with root_node.addChild("plugins") as plugins:
        plugins.addObject('RequiredPlugin', pluginName="""
            Sofa.Component.Constraint.Projective
            Sofa.Component.Engine.Select
            Sofa.Component.LinearSolver.Direct
            Sofa.Component.Mapping.Linear
            Sofa.Component.Mapping.NonLinear
            Sofa.Component.Mass
            Sofa.Component.ODESolver.Backward
            Sofa.Component.SolidMechanics.FEM.Elastic
            Sofa.Component.StateContainer
            Sofa.Component.Topology.Container.Grid
            Sofa.Component.Visual
            SofaMatrix
            SofaMatrix.imgui""")

    root_node.addObject('DefaultAnimationLoop')
    root_node.addObject('VisualGrid')

    # ========================================================================
    # GEOMETRY / PARTITIONING
    # This section only defines the mesh and figures out, for every node,
    # whether it belongs to the "rigid" region or the "deformable" region.
    # No physics happens here -- this is purely bookkeeping/index computation
    # that later sections will use to wire up the mappings correctly.
    # ========================================================================
    with root_node.addChild("geometry") as geometry:
        # Define the full computational domain: a regular 3D grid of nodes.
        grid = geometry.addObject('RegularGridTopology', name="grid", n=[5, 5, 20], xmin=-1.5, xmax=1.5, ymin=-1.5,
                                  ymax=1.5, zmin=0, zmax=20)
        # Define the ROI (Region of Interest): a box selecting which grid nodes
        # fall inside the region that will be made rigid.
        box_roi = geometry.addObject('BoxROI', name="rigid", template="Vec3", box=[[-2, -2, 5], [2, 2, 15]],
                                     position="@grid.position")

        geometry.init()

        # --- Build the lookup tables needed by SubsetMultiMapping below ---
        # BoxROI already split the node indices for us:
        indices_in = box_roi.indices.value  # global indices of nodes INSIDE the box  -> will become rigid
        indices_out = box_roi.indicesOut.value  # global indices of nodes OUTSIDE the box -> will stay deformable
        N = len(grid.position.value)  # total number of nodes in the whole grid

        # For every global node index, determine which "parent" DoF container
        # (subset) it will belong to once we assemble the composite object.
        all_indices = np.arange(0, N)
        mask_in = np.isin(all_indices, indices_in)

        # These IDs must match the order the two inputs are listed in
        # SubsetMultiMapping's "input" argument further down:
        #   input="@.../deformableDoFs  @.../rigidifiedDoFs"
        #          ^ parent_id_out=0     ^ parent_id_in=1
        parent_id_in = 1  # index of "rigidifiedDoFs" in the SubsetMultiMapping input list
        parent_id_out = 0  # index of "deformableDoFs" in the SubsetMultiMapping input list
        parent_ids = np.where(mask_in, parent_id_in, parent_id_out).astype(np.int32)

        # Each subset (deformable / rigid) has its own *local* numbering,
        # starting at 0, independent of the global grid numbering. This array
        # translates each global node index into its position within its subset.
        local_ids = np.full(N, -1, dtype=int)
        local_ids[indices_in] = np.arange(indices_in.size, dtype=np.int32)
        local_ids[indices_out] = np.arange(indices_out.size, dtype=np.int32)

        # SubsetMultiMapping expects a single flat array of (parent_id, local_id)
        # pairs, one pair per node of the final composite object, in the order
        # the composite nodes should appear. We interleave the two arrays here:
        # [parent_0, local_0, parent_1, local_1, parent_2, local_2, ...]
        index_pairs = np.empty(N * 2, dtype=parent_ids.dtype)
        index_pairs[0::2] = parent_ids  # even slots: which subset the node comes from
        index_pairs[1::2] = local_ids  # odd slots: the node's index within that subset

    with root_node.addChild("simulation") as simulation:
        # Time integration and linear solver used for the whole simulation.
        simulation.addObject('EulerImplicitSolver', name="odesolver", rayleighStiffness=0.1, rayleighMass=0.1,
                             printLog=False)
        simulation.addObject('SparseLDLSolver', template="CompressedRowSparseMatrix")
        # Lets you visualize the assembled system matrix. Its size is reduced
        # compared to a fully flexible mesh (deformable DoFs + 6 rigid DoFs
        # only) and you can spot the coupling terms between the deformable
        # part and the rigid body -- see the docstring above for details.
        simulation.addObject('GlobalSystemMatrixImage')

        # ====================================================================
        # DEFORMABLE DOFS SECTION
        # This node does NOT contain any physics (no mass,
        # no force field). It only stores the positions of the "outside the
        # box" nodes and applies a boundary condition that pins some of them
        # in place. Think of it as a plain container of positions that the
        # "composite" node (further below) will read from and push forces
        # back into -- the actual elastic behavior is computed there.
        # ====================================================================
        with simulation.addChild("deformableDoFs") as deformableDoFs:
            # Positions only -- no Mass, no ForceField attached to this node.
            deformableDoFs.addObject('MechanicalObject', template="Vec3", name="deformableDoFs",
                                     position="@../../geometry/rigid.pointsOutROI")
            deformableDoFs.addObject('VisualPointCloud', position="@deformableDoFs.position", sphereRadius=0.2,
                                     drawMode="Sphere", color="navy")
            # Select the nodes at the very base of the structure (z close to 0)
            # and fix them so the whole object is anchored to the ground.
            deformableDoFs.addObject('BoxROI', name="fixed_roi", template="Vec3", box=[[-2, -2, -0.1], [2, 2, 0.1]],
                                     position="@deformableDoFs.position", drawROI=True)
            deformableDoFs.addObject('FixedProjectiveConstraint', name="fixed", indices="@fixed_roi.indices",
                                     showObject=True)

        # ====================================================================
        # RIGID DOFS SECTION
        # This is where the rigidified region actually gets its rigid-body
        # behavior. "rigidDoF" is a true 6-DoF rigid body (3 for position,
        # 3 for orientation) that the solver integrates directly. Every node
        # that was selected as "inside the box" (rigidifiedDoFs, positions
        # only, no physics of its own either) is then forced to move exactly
        # as if it were welded to that single rigid body, via RigidMapping.
        # ====================================================================
        with simulation.addChild("rigidDoFs") as rigidDoFs:
            # The single master rigid body: 6 DoFs (position + orientation).
            rigidDoFs.addObject('MechanicalObject', template="Rigid3", name="rigidDoF",
                                position=[[0, 0, 10, 0, 0, 0, 1]], showObject=True, showObjectScale="1")
            with rigidDoFs.addChild("rigidifiedDoFs") as rigidifiedDoFs:
                # Positions only -- these nodes have no independent motion;
                # RigidMapping below completely determines where they go.
                rigidifiedDoFs.addObject('MechanicalObject', template="Vec3", name="rigidifiedDoFs",
                                         position="@../../../geometry/rigid.pointsInROI")
                rigidifiedDoFs.addObject('VisualPointCloud', position="@rigidifiedDoFs.position", sphereRadius=0.2,
                                         drawMode="Sphere", color="pink")

                # RigidMapping is a one-parent mapping: it forces every node in
                # "rigidifiedDoFs" (the child/output) to follow the rigid
                # motion of "rigidDoF" (the parent/input), just as if it were
                # solidly bolted to it. It also transmits forces the other
                # way: any force applied to a rigidified node is converted
                # into an equivalent force + torque on the master rigid body.
                rigidifiedDoFs.addObject('RigidMapping', globalToLocalCoords=True, input="@../rigidDoF",
                                         output="@rigidifiedDoFs")

        # ====================================================================
        # THE COMPOSITE BODY (Final Assembly)
        # This is where mass and elastic forces are actually defined. We
        # build one combined MechanicalObject containing every node of the
        # original mesh (deformable + rigid, in their original global order),
        # then attach the FEM mass and force field to IT. Forces computed
        # here are automatically redistributed backward through the mappings:
        #   composite -> deformableDoFs  (plain positions, nothing further)
        #   composite -> rigidifiedDoFs -> rigidDoF  (via RigidMapping, turning
        #                                              forces into the 6-DoF
        #                                              force/torque on the
        #                                              rigid body)
        # This is what makes the deformable and rigid regions interact as one
        # physically consistent object.
        # ====================================================================
        with simulation.addChild("composite") as composite:
            # This MechanicalObject holds no explicit positions of its own;
            # they are entirely computed by SubsetMultiMapping below, from its
            # two parent inputs.
            composite.addObject('MechanicalObject', template="Vec3", name="compositeDoFs")

            # CRITICAL STEP: SubsetMultiMapping
            # This mapping has TWO parents (hence "Multi"):
            #   Input 0 (@../deformableDoFs/deformableDoFs): the flexible nodes
            #   Input 1 (@../rigidDoFs/rigidifiedDoFs/rigidifiedDoFs): the rigidified nodes
            # For every node of "compositeDoFs", "indexPairs" tells the mapping
            # which parent to read its position from and which local index to
            # use within that parent (this is exactly the (parent_id, local_id)
            # table we built above in the "geometry" section).
            # Because SubsetMultiMapping works both ways, it also splits the
            # forces computed on "compositeDoFs" (by the mass and force field
            # below) back out to the two parents, which is how the FEM forces
            # end up correctly driving both the free deformable nodes and the
            # rigid body. This backward propagation happens bottom-to-top in
            # the scene graph -- see "Order of mapping operations matters" in
            # the docstring above for why that order is essential for
            # correctness.
            composite.addObject('SubsetMultiMapping', template="Vec3,Vec3",
                                input="@../deformableDoFs/deformableDoFs @../rigidDoFs/rigidifiedDoFs/rigidifiedDoFs",
                                output="@compositeDoFs", applyRestPosition=True,
                                indexPairs=index_pairs)

            # The actual physics: mass and elastic (corotational FEM) forces,
            # computed over the full hexahedral grid topology, applied to the
            # composite node. This is the ONLY place in the whole scene where
            # mass and a force field are defined.
            with composite.addChild("physics") as physics:
                physics.addObject('NodalMassDensity', property=1)
                physics.addObject('FEMMass', template="Vec3,Hexahedron", topology=grid.getLinkPath())
                physics.addObject('CorotationalFEMForceField', template="Vec3,Hexahedron", youngModulus=10000,
                                  poissonRatio=0.45, topology=grid.getLinkPath())
