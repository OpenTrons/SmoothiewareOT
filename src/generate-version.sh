echo `git log --pretty=format:%h -1 | git describe --exact-match`
touch version.cpp