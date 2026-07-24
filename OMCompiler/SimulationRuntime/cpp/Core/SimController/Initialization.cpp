/*
 * This file belongs to the OpenModelica Run-Time System
 *
 * Copyright (c) 1998-2026, Open Source Modelica Consortium (OSMC), c/o Linköpings
 * universitet, Department of Computer and Information Science, SE-58183 Linköping, Sweden. All rights
 * reserved.
 *
 * THIS PROGRAM IS PROVIDED UNDER THE TERMS OF THE BSD NEW LICENSE OR THE
 * AGPL VERSION 3 LICENSE OR THE OSMC PUBLIC LICENSE (OSMC-PL) VERSION 1.8. ANY
 * USE, REPRODUCTION OR DISTRIBUTION OF THIS PROGRAM CONSTITUTES RECIPIENT'S
 * ACCEPTANCE OF THE BSD NEW LICENSE OR THE OSMC PUBLIC LICENSE OR THE AGPL
 * VERSION 3, ACCORDING TO RECIPIENTS CHOICE.
 *
 * The OpenModelica software and the OSMC (Open Source Modelica Consortium) Public License
 * (OSMC-PL) are obtained from OSMC, either from the above address, from the URLs:
 * http://www.openmodelica.org or https://github.com/OpenModelica/ or
 * http://www.ida.liu.se/projects/OpenModelica, and in the OpenModelica distribution. GNU
 * AGPL version 3 is obtained from: https://www.gnu.org/licenses/licenses.html#GPL. The BSD NEW
 * License is obtained from: http://www.opensource.org/licenses/BSD-3-Clause.
 *
 * This program is distributed WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE, EXCEPT AS EXPRESSLY
 * SET FORTH IN THE BY RECIPIENT SELECTED SUBSIDIARY LICENSE CONDITIONS OF
 * OSMC-PL.
 *
 */

/** @addtogroup coreSimcontroller
 *
 *  @{
 */

#include <Core/ModelicaDefine.h>
#include <Core/Modelica.h>
#include <Core/SimController/Initialization.h>

Initialization::Initialization(shared_ptr<ISystemInitialization> system_initialization, shared_ptr<ISolver> solver)
  : _system(system_initialization)
  , _solver(solver)
{
}

Initialization::~Initialization(void)
{
}

void Initialization::initializeSystem()
{
  shared_ptr<IContinuous> continous_system = dynamic_pointer_cast<IContinuous>(_system);
  shared_ptr<IEvent> event_system = dynamic_pointer_cast<IEvent>(_system);
  shared_ptr<IMixedSystem> mixed_system = dynamic_pointer_cast<IMixedSystem>(_system);
  int dim = event_system->getDimZeroFunc();
  bool* conditions0 = new bool[dim];
  bool* conditions1 = new bool[dim];

  _system->setInitial(true);
  //Initialization of continous equations and bounded parameters

  _system->initialize();
  _solver->stateSelection();

  // Reset boolean variables (including when-condition booleans) to allow edge detection
  // during event iteration. initEquations() pre-computes them, which prevents cascading
  // elsewhen clauses from detecting edges during initialization.
  {
    int dimBool = continous_system->getDimBoolean();
    bool* resetBool = new bool[dimBool]();
    continous_system->setBoolean(resetBool);
    delete[] resetBool;
  }
  event_system->saveAll();

  // Event iteration loop for when-clauses during initialization
  _system->setInitial(false);
  bool restart = true;
  int iter = 0;
  while (restart && !(iter++ > 15))
  {
    event_system->getConditions(conditions0);
    continous_system->evaluateAll();
    restart = event_system->checkForDiscreteEvents();
    event_system->getConditions(conditions1);
    event_system->saveAll();

    if (dim > 0)
      restart = restart || !std::equal(conditions1, conditions1 + dim, conditions0);
  }

  if( _solver->stateSelection())
  {
    _system->initEquations();
	continous_system->stepCompleted(0.0);


    /* report a warning about strange start values */
    if(_solver->stateSelection())
      cout << "Cannot initialize the dynamic state selection in an unique way." << std::endl;
  }
  delete[] conditions0;
  delete[] conditions1;
}
/** @} */ // end of coreSimcontroller