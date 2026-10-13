(function () {
  const scrollButton = document.querySelector(".scroll-to-top");
  const navLinks = Array.from(document.querySelectorAll(".site-nav a"));
  const sections = navLinks
    .map((link) => document.querySelector(link.getAttribute("href")))
    .filter(Boolean);

  function updateScrollState() {
    if (!scrollButton) return;
    scrollButton.classList.toggle("is-visible", window.scrollY > 480);
  }

  function updateActiveNav() {
    const current = sections
      .filter((section) => section.getBoundingClientRect().top <= 130)
      .pop();

    navLinks.forEach((link) => {
      const isActive = current && link.getAttribute("href") === `#${current.id}`;
      link.classList.toggle("is-active", Boolean(isActive));
    });
  }

  window.scrollToTop = function scrollToTop() {
    window.scrollTo({ top: 0, behavior: "smooth" });
  };

  document.querySelectorAll("video[data-start]").forEach((video) => {
    const start = Number.parseFloat(video.dataset.start || "0");

    video.addEventListener("loadeddata", () => {
      video.classList.add("is-ready");
    });

    if (Number.isFinite(start) && start > 0) {
      video.addEventListener(
        "loadedmetadata",
        () => {
          if (video.duration > start) {
            video.currentTime = start;
          }
        },
        { once: true },
      );
    }
  });

  function playTeaserVideos() {
    document.querySelectorAll(".teaser-grid video").forEach((video) => {
      video.muted = true;
      video.playsInline = true;
      const playPromise = video.play();
      if (playPromise) {
        playPromise.catch(() => {
          video.controls = false;
        });
      }
    });
  }

  playTeaserVideos();
  window.addEventListener("load", playTeaserVideos, { once: true });

  const revealObserver = "IntersectionObserver" in window
    ? new IntersectionObserver(
        (entries) => {
          entries.forEach((entry) => {
            if (entry.isIntersecting) {
              entry.target.classList.add("is-visible");
              revealObserver.unobserve(entry.target);
            }
          });
        },
        { threshold: 0.12 },
      )
    : null;

  document.querySelectorAll(".reveal").forEach((element) => {
    if (revealObserver) {
      revealObserver.observe(element);
    } else {
      element.classList.add("is-visible");
    }
  });

  const taskTabs = Array.from(document.querySelectorAll(".task-tab"));
  const taskPanels = Array.from(document.querySelectorAll(".task-panel"));

  taskTabs.forEach((tab) => {
    tab.addEventListener("click", () => {
      const task = tab.dataset.task;

      taskTabs.forEach((item) => {
        item.classList.toggle("is-active", item === tab);
      });

      taskPanels.forEach((panel) => {
        const isActive = panel.dataset.taskPanel === task;
        panel.classList.toggle("is-active", isActive);
        const video = panel.querySelector("video");
        if (!isActive && video) {
          video.pause();
        }
      });
    });
  });

  document.querySelectorAll(".clip-button").forEach((button) => {
    button.addEventListener("click", () => {
      const panel = button.closest(".task-panel");
      if (!panel) return;

      const video = panel.querySelector(".task-video");
      const source = video ? video.querySelector("source") : null;
      const label = panel.querySelector(".video-label");
      const title = panel.querySelector("h3");
      const description = panel.querySelector(".task-description");

      panel.querySelectorAll(".clip-button").forEach((item) => {
        item.classList.toggle("is-active", item === button);
      });

      if (label && button.dataset.label) label.textContent = button.dataset.label;
      if (title && button.dataset.title) title.textContent = button.dataset.title;
      if (description && button.dataset.description) {
        description.textContent = button.dataset.description;
      }

      if (video && source && button.dataset.src) {
        video.pause();
        source.src = button.dataset.src;
        if (button.dataset.poster) {
          video.poster = button.dataset.poster;
        }
        video.load();
      }
    });
  });

  window.copyBibTeX = async function copyBibTeX() {
    const bibtex = document.getElementById("bibtex");
    if (!bibtex) return;

    try {
      await navigator.clipboard.writeText(bibtex.textContent.trim());
      const button = Array.from(document.querySelectorAll("button")).find((item) =>
        item.textContent.includes("Copy BibTeX"),
      );
      if (button) {
        const previousText = button.textContent;
        button.textContent = "Copied";
        window.setTimeout(() => {
          button.textContent = previousText;
        }, 1400);
      }
    } catch (error) {
      console.warn("Unable to copy BibTeX", error);
    }
  };

  updateScrollState();
  updateActiveNav();
  window.addEventListener("scroll", () => {
    updateScrollState();
    updateActiveNav();
  }, { passive: true });
})();
