function  [Raw, Sen] = simObservation(SimRob, SimSen, SimLmk)

% SIMOBSERVATION Simulates range infer sensors



Sen = SimSen;

Raw.type = 'simu';


% We need to global reference the robot in order to work with a real
% world. NOT ALL SENSORS SHALL USE THIS. TO FIX
SimRob.state.gt = SimRob.state.gt;

switch Sen.type
    
    % Simone Zandara @ VICOROB
    
    case 'Multibeam2D'
        
        [Raw.data.localCart Raw.data.localPolar] = multibeamComplex2D(SimRob, SimLmk, SimSen);
        
        
    case 'Singlebeam2D'
        %TODO
        
    otherwise
        % Print an error and exit
        error('??? Unknown sensor type ''%s''.',Sen.type);
        
end % end of the "switch" on sensor type


end
