/******************************************************************************
*                              SofaPython3 plugin                             *
*                  (c) 2021 CNRS, University of Lille, INRIA                  *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/

#include <pybind11/pybind11.h>

#include <SofaPython3/SofaBaseTopology/Binding_RegularGridTopology.h>
#include <SofaPython3/SofaBaseTopology/Binding_SparseGridTopology.h>

#include <sofa/component/topology/container/grid/init.h>

namespace py { using namespace pybind11; }

namespace sofapython3
{

PYBIND11_MODULE(SofaBaseTopology, m)
{
    m.doc() = "Implements some topology component (Regular and SparseGridTopology)";

    sofa::component::topology::container::grid::init();

    moduleAddRegularGridTopology(m);
    moduleAddSparseGridTopology(m);
}

} // namespace sofapython3
