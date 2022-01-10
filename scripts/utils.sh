


checkDeps() {
  sudo apt update
  # loop over all dependencies passed in and install them
  for dep; do sudo apt install "$dep" -y; done
}

checkFileExists() {
  for path; do
    test -e $path || echo "File does not exist ${path}" && exit 2
  done
}

getVideoPath() {
  # if argument is present, use that for filename
  if [[ -z "$1" ]]; then
    # default to live camera on device
    video=""
  else
    video="$1"
  fi
  echo $video
}
