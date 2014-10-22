#!/usr/bin/env bash

# This file will install google repo tool for galt deployment
CURRENT_DIR=$(pwd)
REPO_DIR=~/bin

if [[ ! -d ${REPO_DIR} ]]; then
    echo "Creating folder ${REPO_DIR}."
    mkdir ${REPO_DIR}
fi

PATH=${REPO_DIR}:${PATH}
curl https://storage.googleapis.com/git-repo-downloads/repo > ${REPO_DIR}/repo
chmod a+x ${REPO_DIR}/repo
echo "google repo installed to ${REPO_DIR}/repo."

echo "now cloning Galt and dependencies."
CLONE_DIR=~/Workspace/repo
NOUKA_DIR=nouka

if [[ ! -d ${CLONE_DIR} ]]; then
    echo "Creating directory ${CLONE_DIR}."
    mkdir -p ${CLONE_DIR}
fi

cd ${CLONE_DIR}
if [[ ! -d ${NOUKA_DIR} ]]; then
    mkdir ${NOUKA_DIR}
fi
cd ${NOUKA_DIR}
${REPO_DIR}/repo init -u  https://github.com/versatran01/galt_repo.git
${REPO_DIR}/repo sync
echo "All depencies clone to ${CLONE_DIR}/${NOUKA_DIR}."

cd ${CURRENT_DIR}

