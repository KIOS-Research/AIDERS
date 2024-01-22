#!/bin/bash
# Repack the .git folder: This reorganizes all existing objects in a single package more efficiently
git repack -a -d --depth=250 --window=250
# Prune and garbage collect: This removes unnecessary files and optimizes the local repository
git gc --auto
# Prune remote tracking branches: This deletes all stale tracking branches which have already been removed at origin but are still locally available in remotes/origin
git remote prune origin