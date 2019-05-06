function []=AcquireSatellite();
% %% Acquire the Satellite that is visible for duration of time
% % SatStateMod = SatState;
% % [N,Svid_Step] = size(SatStateMod);
% % Svid_list = SatStateMod{1,1}(:,1);
% 
% % for i = 1:1:Svid_Step
% %      if numel(Svid_list) >= 4
% %         Svid = SatStateMod{1,i}(:,1);
% %         check_Svid = ismember(Svid_list,Svid);
% %         if all(check_Svid == 1)
% %             Svid_list = Svid_list;
% %         elseif numel(Svid_list) > 4
% %             Svid_list = Svid_list.*check_Svid;
% %             Svid_list = Svid_list(find(Svid_list ~= 0)); 
% %         else
% %             break;
% %         end 
% %     end  
% % end
% % 
% % for ii = 1:1:Svid_Step
% %     check_Sat = ismember(SatStateMod{1,ii}(:,1),Svid_list);
% %     [r,c] = find(check_Sat == 0);
% %     SatStateMod{1,ii}(r,:) = [];
% % end