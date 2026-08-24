/* Page chrome: sticky-header state, scrollspy, reveal-on-scroll, footer year.
   Smooth scrolling is handled by CSS (`scroll-behavior: smooth`), so there's
   no click interception here — external links in the nav keep working. */

document.addEventListener('DOMContentLoaded', () => {

  const header = document.querySelector('.site-header');
  const navLinks = Array.from(document.querySelectorAll('.nav-links a'));
  const sections = navLinks
    .map((a) => document.querySelector(a.getAttribute('href')))
    .filter(Boolean);

  // Footer year
  const year = document.getElementById('year');
  if (year) year.textContent = String(new Date().getFullYear());

  // Header gets a hairline border once you've scrolled off the top.
  let ticking = false;
  const onScroll = () => {
    if (ticking) return;
    ticking = true;
    requestAnimationFrame(() => {
      header.classList.toggle('is-stuck', window.scrollY > 8);
      ticking = false;
    });
  };
  window.addEventListener('scroll', onScroll, { passive: true });
  onScroll();

  // Scrollspy — highlight whichever section owns the upper third of the viewport.
  if (sections.length && 'IntersectionObserver' in window) {
    const visible = new Map();
    const spy = new IntersectionObserver(
      (entries) => {
        for (const entry of entries) visible.set(entry.target, entry.intersectionRatio);

        let best = null;
        let bestRatio = 0;
        for (const [el, ratio] of visible) {
          if (ratio > bestRatio) { bestRatio = ratio; best = el; }
        }

        navLinks.forEach((link) => {
          const active = best && link.getAttribute('href') === '#' + best.id;
          link.classList.toggle('active', Boolean(active));
        });
      },
      { rootMargin: '-15% 0px -55% 0px', threshold: [0, 0.25, 0.5, 0.75, 1] }
    );
    sections.forEach((s) => spy.observe(s));
  }

  // Reveal cards and section headers as they scroll in.
  const revealables = document.querySelectorAll('.section-head, .card, .sim-frame, .prose, .contact-form, .more-repos');
  if ('IntersectionObserver' in window) {
    const reveal = new IntersectionObserver(
      (entries, obs) => {
        entries.forEach((entry) => {
          if (!entry.isIntersecting) return;
          entry.target.classList.add('is-visible');
          obs.unobserve(entry.target);
        });
      },
      { rootMargin: '0px 0px -8% 0px', threshold: 0.05 }
    );
    revealables.forEach((el, i) => {
      el.classList.add('reveal');
      // Slight stagger so a row of cards cascades instead of popping at once.
      el.style.transitionDelay = (i % 4) * 60 + 'ms';
      reveal.observe(el);
    });
  }
});
