# Contributing to the Rust DW1000 Driver

Thank you for considering to work on this project. This document will give you some pointers.


## Opening issues

If you found a problem, please [open an issue] on the [GitHub repository]. If you're not sure whether you found a real problem or not, just open an issue anyway. We'd rather close a few invalid issues than miss anything relevant.


## Contributing changes

If you want to fix an issue or implement a new feature, just fork the repository, make your change, and create a [pull request]. If you're concerned that your change might not be accepted, feel free to [open an issue] to discuss things beforehand.

If you're having any problems with completing your change, feel free to open a pull request anyway and ask any questions there. We're happy to help you get your changes across the finish line.


## Release Procedure

This section is intended for project maintainers only. It assumes that you can push to the repository (here called `upstream`), but primarily work on your own fork (`origin`),

1. Create a branch for the release (replace x.y.z with actual version)
```
$ git checkout -b release-x.y.z
```

2. Update `CHANGELOG.md`

3. Update version in `Cargo.toml`

4. Update versions in README.md, if version bump is major or minor

5. Commit these changes

6. Open pull request; state your intention to release a new version
```
$ git push -u origin release-x.y.z
# Open pull request
```

7. Review pull request yourself or wait for reviews, as appropriate

8. Publish the crate
```
$ cargo publish
```

9. Merge pull request and update your local repository
```
$ git checkout master
$ git pull upstream master
```

10. Tag the release (latest commit is assumed to be the appropriate one here)
```
$ git tag vx.y.z
$ git push --tag upstream
```

11. Update dependency on `dwm1001` crate
The `dwm1001` crate is used for the examples. If this was a breaking release, release a new `dwm1001` version too, then update the dependency in this crate and release that as a new patch version.


[open an issue]: https://github.com/braun-robotics/rust-dw1000/issues/new
[GitHub repository]: https://github.com/braun-robotics/rust-dw1000
[pull request]: https://github.com/braun-robotics/rust-dw1000/pulls
