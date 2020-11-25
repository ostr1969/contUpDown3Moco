set -g prefix C-a
bind C-a send-prefix
unbind C-b
is_vim="ps -o state= -o comm= -t '#{pane_tty}' \
  | grep -iqE '^[^TXZ ]+ +(\\S+\\/)?g?(view|n?vim?x?)(diff)?$'"
bind-key -n Pageup if-shell "$is_vim" "send-keys PPage"  "copy-mode -u"

bind-key -n C-Up select-pane -U
bind -n C-Space select-pane -t :.+
bind -n M-Space next-window
bind-key      -n   M-Right                  next-window 
bind-key    -n     M-Left              previous-window

bind-key -n C-Down select-pane -D
bind-key -n C-Right select-pane -R
bind-key -n C-Left select-pane -L
set-window-option -g window-status-current-bg red
set -g mode-keys vi
set -g status-keys vi
set -g mouse off
set -g monitor-activity on
set -g base-index 1
bind-key    -T prefix       |     split-window -h
bind-key    -T prefix       -     split-window 
bind r source-file ${HOME}/.tmux.conf \; display-message "source-file reloaded"
bind-key v  choose-window "join-pane -h -t '%%'"
bind-key h  choose-window "join-pane -v -t '%%'"
bind-key -n C-Up select-pane -U
bind-key -n C-Down select-pane -D
bind-key -n C-Right select-pane -R
bind-key -n C-Left select-pane -L
#bind -n Pageup copy-mode -u
set -g history-limit 30000
bind-key -T prefix H send-keys "Mini help for tmux"\;
#display-message "C-U/D/L/R move to other pane"
