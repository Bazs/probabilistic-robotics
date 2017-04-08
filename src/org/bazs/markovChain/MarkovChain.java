package org.bazs.markovChain;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;

public class MarkovChain<State extends Enum<State>>
{
   private final Map<State, RandomDecider<State>> _transitionTable;

   private State _currentState;

   public MarkovChain(State initialState, Map<State, RandomDecider<State>> transitionTable)
   {
      if ((transitionTable.size() != EnumSet.allOf(initialState.getClass()).size()))
      {
         throw new IllegalArgumentException("The decision table is not complete.");
      }

      _currentState = initialState;
      _transitionTable = new HashMap<>();
      _transitionTable.putAll(transitionTable);
   }

   public State doTransition()
   {
      _currentState = _transitionTable.get(_currentState).nextDecision();
      return _currentState;
   }

   public void setState(State state)
   {
      _currentState = state;
   }
}
