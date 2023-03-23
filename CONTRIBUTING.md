# How to Contribute

Thank you for your interest and time spent contributing!  
We welcome contributions from everyone, and to ensure our community stays open and healthy
we adhere to the [Contributor Covenant](https://www.contributor-covenant.org/), a widely
used [code of conduct](./CODE_OF_CONDUCT.md) adopted by many other communities such as
Linux, Autoware, GitLab.

## Communication

First, please read through our [code of conduct](./CODE_OF_CONDUCT.md),
as we expect all our contributors to follow it.

Second, before starting on a project that you intend to contribute to any of our
projects, we **strongly** recommend posting on the repository's
[Issues page](https://github.com/mangdangroboticsclub/mini_pupper/issues) or
[Discord](https://discord.gg/xJdt3dHBVw) and
briefly outlining the changes you plan to make.  
This will enable us to provide
some context that may be helpful for you. This could range from advice and
feedback on how to optimally perform your changes or reasons for not doing it.

## Contribution Workflow

1. [Create an issue](https://github.com/mangdangroboticsclub/mini_pupper/issues) or [post on Discord](https://discord.gg/xJdt3dHBVw) defining your intended contribution
2. Create a fork
    * For more information about the fork-and-pull model, see the [GitHub Docs](https://docs.github.com/en/get-started/quickstart/contributing-to-projects?tool=webui&platform=linux).
3. Write code
4. Create a pull request
    * Fill the template
    * For more information about the fork-and-pull model, see the [GitHub Docs](https://docs.github.com/en/get-started/quickstart/contributing-to-projects?tool=webui&platform=linux).
5. Finish a pull request
    * In order for a pull request to be merged to mini_pupper, it must meet the following criteria:
        * All discussions on the pull request must be resolved.
        * All items of the pull request checklist are checked off.
        * CI jobs for the pull request must have passed successfully.

### Pull Requests

For Pull Requests, please target the `main` branch for any contributions.
To contribute, please check out the `main` branch, and then create your feature
branch from there:

```sh
git checkout main                # start with the main branch
git pull origin main             # pull remote repo changes
git checkout your-feature-branch # create your feature branch
```
Then when you submit a Pull Request, please select the `main` branch to request
to merge your commits.

If you are interested in understanding this development style a bit further,
we follow the [GitHub Flow](https://docs.github.com/en/get-started/quickstart/github-flow)
model of branching.

## Guildlines for Development

### License

mini_pupper_ros is licensed under the Apache 2.0 License, and thus all contributions will be licensed as such
as per clause 5 of the Apache 2.0 License:

~~~
5. Submission of Contributions. Unless You explicitly state otherwise,
   any Contribution intentionally submitted for inclusion in the Work
   by You to the Licensor shall be under the terms and conditions of
   this License, without any additional terms or conditions.
   Notwithstanding the above, nothing herein shall supersede or modify
   the terms of any separate license agreement you may have executed
   with Licensor regarding such Contributions.
~~~
