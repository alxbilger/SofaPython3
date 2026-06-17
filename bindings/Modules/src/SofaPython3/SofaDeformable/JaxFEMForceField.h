#pragma once
#include <sofa/config.h>
#include <sofa/core/behavior/ForceField.h>
#include <sofa/core/behavior/TopologyAccessor.h>
#include <sofa/core/visual/DrawMesh.h>

namespace sofapython3
{

template <class TDataTypes, class TElementType>
class JaxFEMForceField :
    public sofa::core::behavior::ForceField<TDataTypes>,
    public sofa::core::behavior::TopologyAccessor
{
public:
    SOFA_CLASS(
        SOFA_TEMPLATE2(JaxFEMForceField, TDataTypes, TElementType),
        sofa::core::behavior::ForceField<TDataTypes>
    );

    using DataTypes = TDataTypes;

    std::string getClassName() const override;
    void init() override;

    using sofa::core::behavior::ForceField<TDataTypes>::addForce;
    void addForce(
       const sofa::core::MechanicalParams* mparams,
       sofa::DataVecDeriv_t<DataTypes>& data_f,
       const sofa::DataVecCoord_t<DataTypes>& data_x,
       const sofa::DataVecDeriv_t<DataTypes>& data_v) override;

    void draw(const sofa::core::visual::VisualParams* vparams) override;


    sofa::Data<sofa::Real_t<DataTypes>> d_elementSpace;

protected:
    static constexpr sofa::Size spatial_dimensions = DataTypes::spatial_dimensions;
    static constexpr sofa::Size NumberOfNodesInElement = TElementType::NumberOfNodes;
    static constexpr sofa::Size NumberOfDofsInElement = NumberOfNodesInElement * spatial_dimensions;

    using ElementGradient = sofa::type::Vec<NumberOfDofsInElement, sofa::Real_t<DataTypes>>;

    sofa::core::visual::DrawElementMesh<TElementType> m_drawMesh;

    sofa::type::vector<ElementGradient> m_elementForce;
    sofa::type::vector<ElementGradient> m_elementDForce;

    void computeElementsForces(const sofa::core::MechanicalParams* mparams,
        sofa::type::vector<ElementGradient>& f,
        const sofa::VecCoord_t<DataTypes>& x);

    JaxFEMForceField();
};

} // namespace sofapython3
