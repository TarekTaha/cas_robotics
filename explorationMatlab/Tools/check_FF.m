%% check_FF (S)
        function result = check_FF(t,ellip_cent,ellip_params,points) 
        %%
        % *Description:*  This function takes a transform and an ellipse definition
        % and translates the points back to the original cordinate frame of the
        % ellipse. Note the reason for this is because the ellipse has been
        % translated because of the movement of the link of the robot which it
        % encompases. This function simply returns true/false as to whether there
        % are any points inside the ellipse in question. 
        % _Note_ eventually I want to put in unknown points and an acceptable limit 

        %% Function Call
        % 
        % *Inputs:* 
        %
        % _t_ (4*4 double) this is the transform of the arm,
        %
        % _ellip_cent_ (struct) center of the ellipse
        %
        % _ellip_params_ (struct) parameters of the ellipse
        %
        % _points_ (3*m double) holds the points in space
        %
        % *Returns:* 
        %
        % _result_ (bin) =1 if ok, =0 if NOT ok     
        
            %assumed there's nothing to collide with
            result=true;

            %% As long as there are some points
            % $$P \neq \emptyset $$
            numpoints=size(points,1);
            if numpoints>0

            % Translate Points back to coordinate frame of ellipse
            %we want to inverse translate the points IE leave the elispes where they were to start off with and translate the world around them
                translated_points_1=points(:,1)-t(1,4);
                translated_points_2=points(:,2)-t(2,4);
                translated_points_3=points(:,3)-t(3,4);       


            % Check if there are points in ellipse           
                translated_points_1_t=translated_points_1*t(1,1)+translated_points_2*t(1,2)+translated_points_3*t(1,3);

                index_1=find(abs(translated_points_1_t-ellip_cent(1))<=abs(ellip_params(1)));    
                if ~isempty(index_1)
                    translated_points_2_t=translated_points_1*t(2,1)+translated_points_2*t(2,2)+translated_points_3*t(2,3);        

                    index_2=find(abs(translated_points_2_t(index_1)-ellip_cent(2))<=abs(ellip_params(2)));    
                    if ~isempty(index_2)
                        translated_points_3_t=translated_points_1*t(3,1)+translated_points_2*t(3,2)+translated_points_3*t(3,3);

                        index_3=find(abs(translated_points_3_t(index_1(index_2))-ellip_cent(3))<=abs(ellip_params(3)));    
                        if ~isempty(index_3)
                            if ~isempty(find(((translated_points_1_t(index_1(index_2(index_3)))-ellip_cent(1)).^2)/ellip_params(1)^2+...
                                             ((translated_points_2_t(index_1(index_2(index_3)))-ellip_cent(2)).^2)/ellip_params(2)^2+...
                                             ((translated_points_3_t(index_1(index_2(index_3)))-ellip_cent(3)).^2)/ellip_params(3)^2<=1,1))
                                result=false; % THERE IS some point is inside
                            end
                        end
                    end
                end

            end
        end