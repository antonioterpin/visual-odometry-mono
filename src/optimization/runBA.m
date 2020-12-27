function hidden_state = runBA(hidden_state, observations, K)


error_terms = @(hidden_state) baError_(hidden_state, observations, K);
options = optimoptions(@lsqnonlin, 'Display', 'iter', 'MaxIter', 20, 'JacobPattern', J);
hidden_state = lsqnonlin(error_terms, hidden_state, [], [], options);

end

function e = baError_(hiddenState, observedState, K)
    
end