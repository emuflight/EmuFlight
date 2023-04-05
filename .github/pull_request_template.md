### Thank you for contributing to EmuFlight!

### Important considerations when opening a pull request:

1. Please code all changes into a new branch based on master and Pull-Request from this branch, do not modify `master`.

2. Pull-Requests will only be accepted if they are opened against the `master` branch. Pull-Requests opened against other branches without prior consent from the maintainers will be closed;

3. Please follow the coding style guidelines: https://github.com/emuflight/EmuFlight/blob/master/docs/development/CodingStyle.md

4. Keep your Pull-Requests as small and concise as possible. One pull request should only ever add / update one feature. If the change that you are proposing has a wider scope, consider splitting it over multiple Pull-Requests. In particular, Pull-Requests that combine changes to features and one or more new targets are not acceptable.

5. Ideally, a pull request should contain only one commit, with a descriptive message. If your changes use more than one commit, rebase / squash them into one commit before submitting a pull request. If you need to amend your pull request, make sure that the additional commit has a descriptive message, or - even better - use `git commit --amend` to amend your original commit.

6. All Pull-Requests are reviewed. Be ready to receive constructive criticism, and to learn and improve your coding style. Also, be ready to clarify anything that isn't already sufficiently explained in the code and text of the pull request, and to defend your ideas.

7. If your pull request is a fix for one or more issues that are open in GitHub, add a comment to your pull request, and add the issue numbers of the issues that are fixed in the form `Fixes #<issue number>`. This will cause the issues to be closed when the pull request is merged;

8. All PR's should pass the following commands as well as Codacy.com checks.
* `make EXTRA_FLAGS=-Werror checks`
* `make EXTRA_FLAGS=-Werror test-all`
* `make EXTRA_FLAGS=-Werror all`

9. Remove this Text :).

### Any questions?
* Please join our discord server: https://discord.gg/BWqgBg3
