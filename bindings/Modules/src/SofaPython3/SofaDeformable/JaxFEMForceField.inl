#pragma once

#include <SofaPython3/PythonEnvironment.h>
#include <SofaPython3/SofaDeformable/JaxFEMForceField.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/fem/FiniteElement.h>

namespace sofapython3
{
namespace py { using namespace pybind11; }

template <class TDataTypes, class TElementType>
JaxFEMForceField<TDataTypes, TElementType>::JaxFEMForceField()
    : d_elementSpace(initData(&d_elementSpace, static_cast<sofa::Real_t<DataTypes>>(0.125), "elementSpace", "When rendering, the space between elements"))
{
    d_elementSpace.setGroup("Visualization");
}

template <class TDataTypes, class TElementType>
std::string JaxFEMForceField<TDataTypes, TElementType>::getClassName() const
{
    PythonEnvironment::gil acquire {"getClassName"};

    // Get the actual class name from python.
    return py::str(py::cast(this).get_type().attr("__name__"));
}

template <class TDataTypes, class TElementType>
void JaxFEMForceField<TDataTypes, TElementType>::init()
{
    sofa::core::behavior::ForceField<DataTypes>::init();

    if (!this->isComponentStateInvalid())
    {
        TopologyAccessor::init();
    }

    PythonEnvironment::gil acquire {"JaxFEMForceField::init"};

    // initialize JAX if not already done
    try
    {
        py::module_ jax = py::module_::import("jax");
        py::module_ jnp = py::module_::import("jax.numpy");

        py::object self = py::cast(this);
        if (py::hasattr(self, "energy"))
        {
            py::object energy_func = self.attr("energy");
            py::object grad_func = jax.attr("grad")(energy_func);

            // gradient = jnp.grad(energy())
            // gvmap = jnp.jit(jnp.vmap(gradient))
            py::object vmap_grad = jax.attr("vmap")(grad_func);
            self.attr("_jit_gvmap") = jax.attr("jit")(vmap_grad);
        }
        else
        {
            msg_warning() << "No 'energy' method found in Python class. JAX force computation will be unavailable.";
        }
    }
    catch (py::error_already_set& e)
    {
        msg_error() << "JAX initialization failed: " << e.what();
    }
}

template <class TDataTypes, class TElementType>
void JaxFEMForceField<TDataTypes, TElementType>::addForce(
    const sofa::core::MechanicalParams *mparams,
    sofa::DataVecDeriv_t<DataTypes> &data_f,
    const sofa::DataVecCoord_t<DataTypes> &data_x,
    const sofa::DataVecDeriv_t<DataTypes> &data_v)
{
    SOFA_UNUSED(mparams);
    SOFA_UNUSED(data_v);

    auto x = sofa::helper::getReadAccessor(data_x);

    if (this->l_topology == nullptr) return;

    const auto& elements = sofa::fem::FiniteElement<TElementType, TDataTypes>::getElementSequence(*this->l_topology);
    m_elementForce.resize(elements.size());

    this->computeElementsForces(mparams, m_elementForce, x.ref());

    auto f = sofa::helper::getWriteOnlyAccessor(data_f);
    if (f.size() < x.size())
    {
        f.resize(x.size());
    }

    // dispatch the element force to the degrees of freedom.
    // this operation is done outside the compute strategy because it is not thread-safe.
    // dispatchElementForcesToNodes(elements, f.wref());
}

template <class TDataTypes, class TElementType>
void JaxFEMForceField<TDataTypes, TElementType>::computeElementsForces(
    const sofa::core::MechanicalParams *mparams,
    sofa::type::vector<ElementGradient> &f, const sofa::VecCoord_t<DataTypes> &x)
{
    // class StVenantKirchhoff(TetrahedronJaxFEMForceFieldVec3d):
    // def __init__(self):
    //     self.lambda = 1000
    //     self.mu = 10
    // def energy(F):
    //     E = (1/2)(F.T @ F - I)
    //     return self.lambda * E.trace() * I + 2 * self.mu * E

    py::exec("gvmap()");

    // vfunc = jnp.vmap(g())
    //call to jax jit-compiled vectorized function vfunc
    // vfunc(f, x)
}

template <class TDataTypes, class TElementType>
void JaxFEMForceField<TDataTypes, TElementType>::draw(
    const sofa::core::visual::VisualParams* vparams)
{
    if (!vparams->displayFlags().getShowForceFields())
        return;

    if (!this->l_topology)
        return;

    const auto stateLifeCycle = vparams->drawTool()->makeStateLifeCycle();

    if (vparams->displayFlags().getShowWireFrame())
        vparams->drawTool()->setPolygonMode(0, true);

    const auto& x = this->mstate->read(sofa::core::vec_id::read_access::position)->getValue();

    m_drawMesh.elementSpace = d_elementSpace.getValue();
    m_drawMesh.drawAllElements(vparams->drawTool(), x, this->l_topology.get());
}

} // namespace sofapython3
