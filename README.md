# Controls
Code for controlling the boat's motion. Vroom vroom.

## Changing the code
To make changes to the code, clone the repo by selecting '<> Code' and copying the HTTPS link. Open VSCode, select 'Clone Git Repository...', copy the link, and follow the instructions.

Check out to a new branch to make changes:
```
git checkout <branchname>
```
While working on your branch, changes might be made to the master branch. You can check for changes with ```git status```. To incorporate these changes in your branch, first stage your changes:
```
git add .
git commit -m "some message about your changes"
```
Then checkout to main, pull the changes, check back to your branch, and rebase with main:
```
git checkout master
git pull
git checkout <branchname>
git rebase master
```
You will be asked to reconcile any differences between your branch and the master branch. **Make sure to do this rebasing process before you make any pull requests**.

You can push your changes to your branch after commiting them, and should do this before making a pull request:
```
git push origin <branchname>
```
To make changes to the master branch, you must submit a pull request. There are a few ways to do this.
1) After pushing to your branch, a link will be output in terminal: "Create a pull request for 'branchname' on Github by visiting: ...". You can follow this link in your browser.
2) Or, go to the Github website for the repo. Click on 'Pull requests'. Click 'New pull request' and select your branch. Then click 'Create pull request'.
Please make sure your pull request has a fitting title and add a description describing your changes. Focus on how your changes impact the overall behavior of the code. You must also add two reviewers. At least one of these reviewers must be a lead (FA23: Amelia Kovacs, Eric Zhang, Taha Jafry, Kevin Cui) and the 'point person' of the project (Will or Jeffrey for FA23).
