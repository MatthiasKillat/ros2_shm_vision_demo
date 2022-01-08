


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
